#include <bitbots_vision/onnx_layernorm_decomposition.hpp>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace bitbots_vision::onnx_rewrite {

namespace {

// ---------------------------------------------------------------------------
// Protobuf wire format helpers
// ---------------------------------------------------------------------------

enum WireType : uint32_t { kVarint = 0, kFixed64 = 1, kLengthDelimited = 2, kFixed32 = 5 };

// Field numbers from onnx.proto
namespace model_field {
constexpr uint32_t kGraph = 7;
constexpr uint32_t kOpsetImport = 8;
}  // namespace model_field
namespace opset_field {
constexpr uint32_t kDomain = 1;
constexpr uint32_t kVersion = 2;
}  // namespace opset_field
namespace graph_field {
constexpr uint32_t kNode = 1;
constexpr uint32_t kInitializer = 5;
constexpr uint32_t kInput = 11;
constexpr uint32_t kOutput = 12;
constexpr uint32_t kValueInfo = 13;
}  // namespace graph_field
namespace value_info_field {
constexpr uint32_t kName = 1;
constexpr uint32_t kType = 2;
}  // namespace value_info_field
namespace type_field {
constexpr uint32_t kTensorType = 1;
constexpr uint32_t kElemType = 1;
}  // namespace type_field
namespace node_field {
constexpr uint32_t kInput = 1;
constexpr uint32_t kOutput = 2;
constexpr uint32_t kName = 3;
constexpr uint32_t kOpType = 4;
constexpr uint32_t kAttribute = 5;
constexpr uint32_t kDomain = 7;
}  // namespace node_field
namespace attribute_field {
constexpr uint32_t kName = 1;
constexpr uint32_t kF = 2;
constexpr uint32_t kI = 3;
constexpr uint32_t kT = 5;
constexpr uint32_t kInts = 8;
constexpr uint32_t kType = 20;
}  // namespace attribute_field
namespace attribute_type {
constexpr uint64_t kInt = 2;
constexpr uint64_t kTensor = 4;
constexpr uint64_t kInts = 7;
}  // namespace attribute_type
namespace tensor_field {
constexpr uint32_t kDims = 1;
constexpr uint32_t kDataType = 2;
constexpr uint32_t kName = 8;
constexpr uint32_t kRawData = 9;
}  // namespace tensor_field
namespace tensor_type {
constexpr uint64_t kFloat = 1;
constexpr uint64_t kInt64 = 7;
}  // namespace tensor_type

/// One top-level field of a protobuf message. `payload` holds the decoded
/// bytes for length-delimited fields; `raw` always spans the complete
/// encoded field (tag included) so unmodified fields can be copied verbatim.
struct Field {
  uint32_t number{0};
  WireType wire_type{kVarint};
  uint64_t value{0};  // varint / fixed32 / fixed64 value
  std::string_view payload;
  std::string_view raw;
};

class Reader {
 public:
  explicit Reader(std::string_view data) : data_(data) {}

  bool done() const { return pos_ >= data_.size(); }

  Field next() {
    const size_t start = pos_;
    const uint64_t key = read_varint();
    Field field;
    field.number = static_cast<uint32_t>(key >> 3);
    field.wire_type = static_cast<WireType>(key & 0x7);
    switch (field.wire_type) {
      case kVarint:
        field.value = read_varint();
        break;
      case kFixed64:
        field.value = read_fixed(8);
        break;
      case kFixed32:
        field.value = read_fixed(4);
        break;
      case kLengthDelimited: {
        const uint64_t len = read_varint();
        require(len);
        field.payload = data_.substr(pos_, len);
        pos_ += len;
        break;
      }
      default:
        throw std::runtime_error("ONNX protobuf: unsupported wire type " + std::to_string(field.wire_type));
    }
    field.raw = data_.substr(start, pos_ - start);
    return field;
  }

 private:
  void require(uint64_t n) const {
    if (n > data_.size() - pos_) {
      throw std::runtime_error("ONNX protobuf: unexpected end of data");
    }
  }

  uint64_t read_varint() {
    uint64_t result = 0;
    for (int shift = 0; shift < 64; shift += 7) {
      require(1);
      const auto byte = static_cast<uint8_t>(data_[pos_++]);
      result |= static_cast<uint64_t>(byte & 0x7F) << shift;
      if ((byte & 0x80) == 0) {
        return result;
      }
    }
    throw std::runtime_error("ONNX protobuf: malformed varint");
  }

  uint64_t read_fixed(int bytes) {
    require(static_cast<uint64_t>(bytes));
    uint64_t result = 0;
    for (int i = 0; i < bytes; ++i) {
      result |= static_cast<uint64_t>(static_cast<uint8_t>(data_[pos_++])) << (8 * i);
    }
    return result;
  }

  std::string_view data_;
  size_t pos_{0};
};

void write_varint(std::string& out, uint64_t value) {
  while (value >= 0x80) {
    out.push_back(static_cast<char>((value & 0x7F) | 0x80));
    value >>= 7;
  }
  out.push_back(static_cast<char>(value));
}

void write_tag(std::string& out, uint32_t number, WireType wire_type) {
  write_varint(out, (static_cast<uint64_t>(number) << 3) | wire_type);
}

void write_varint_field(std::string& out, uint32_t number, uint64_t value) {
  write_tag(out, number, kVarint);
  write_varint(out, value);
}

void write_bytes_field(std::string& out, uint32_t number, std::string_view bytes) {
  write_tag(out, number, kLengthDelimited);
  write_varint(out, bytes.size());
  out.append(bytes.data(), bytes.size());
}

// ---------------------------------------------------------------------------
// ONNX node model
// ---------------------------------------------------------------------------

struct Node {
  std::vector<std::string> inputs;
  std::vector<std::string> outputs;
  std::string name;
  std::string op_type;
  std::string domain;
  std::optional<int64_t> axis;
  std::optional<float> epsilon;
  /// `to` attribute of Cast nodes
  std::optional<uint64_t> cast_to;
  /// Element type of the `value` attribute of Constant nodes
  std::optional<uint64_t> value_type;
  bool has_other_attributes{false};
};

/// Element type (`TensorProto.DataType`) stored in a serialized TensorProto.
std::optional<uint64_t> tensor_data_type(std::string_view tensor) {
  Reader reader(tensor);
  while (!reader.done()) {
    const Field field = reader.next();
    if (field.number == tensor_field::kDataType && field.wire_type == kVarint) {
      return field.value;
    }
  }
  return std::nullopt;
}

Node parse_node(std::string_view payload) {
  Node node;
  Reader reader(payload);
  while (!reader.done()) {
    const Field field = reader.next();
    switch (field.number) {
      case node_field::kInput:
        node.inputs.emplace_back(field.payload);
        break;
      case node_field::kOutput:
        node.outputs.emplace_back(field.payload);
        break;
      case node_field::kName:
        node.name = std::string(field.payload);
        break;
      case node_field::kOpType:
        node.op_type = std::string(field.payload);
        break;
      case node_field::kDomain:
        node.domain = std::string(field.payload);
        break;
      case node_field::kAttribute: {
        std::string attr_name;
        std::optional<uint64_t> int_value;
        std::optional<uint32_t> float_bits;
        std::optional<uint64_t> tensor_type;
        Reader attr_reader(field.payload);
        while (!attr_reader.done()) {
          const Field attr_field = attr_reader.next();
          if (attr_field.number == attribute_field::kName) {
            attr_name = std::string(attr_field.payload);
          } else if (attr_field.number == attribute_field::kI && attr_field.wire_type == kVarint) {
            int_value = attr_field.value;
          } else if (attr_field.number == attribute_field::kF && attr_field.wire_type == kFixed32) {
            float_bits = static_cast<uint32_t>(attr_field.value);
          } else if (attr_field.number == attribute_field::kT && attr_field.wire_type == kLengthDelimited) {
            tensor_type = tensor_data_type(attr_field.payload);
          }
        }
        if (attr_name == "to" && int_value) {
          node.cast_to = *int_value;
        } else if (attr_name == "value" && tensor_type) {
          node.value_type = *tensor_type;
        } else if (attr_name == "axis" && int_value) {
          node.axis = static_cast<int64_t>(*int_value);
        } else if (attr_name == "epsilon" && float_bits) {
          float eps;
          std::memcpy(&eps, &*float_bits, sizeof(eps));
          node.epsilon = eps;
        } else if (attr_name != "stash_type") {
          // stash_type only selects the internal compute precision, which
          // the decomposition does not need to honor.
          node.has_other_attributes = true;
        }
        break;
      }
      default:
        break;
    }
  }
  return node;
}

std::string encode_ints_attribute(const std::string& name, const std::vector<int64_t>& values) {
  std::string attr;
  write_bytes_field(attr, attribute_field::kName, name);
  // onnx.proto is proto2, where repeated scalars are not packed by default.
  for (const int64_t v : values) {
    write_varint_field(attr, attribute_field::kInts, static_cast<uint64_t>(v));
  }
  write_varint_field(attr, attribute_field::kType, attribute_type::kInts);
  return attr;
}

std::string encode_int_attribute(const std::string& name, int64_t value) {
  std::string attr;
  write_bytes_field(attr, attribute_field::kName, name);
  write_varint_field(attr, attribute_field::kI, static_cast<uint64_t>(value));
  write_varint_field(attr, attribute_field::kType, attribute_type::kInt);
  return attr;
}

std::string encode_tensor(uint64_t data_type, const std::vector<int64_t>& dims, std::string_view raw_data) {
  std::string tensor;
  for (const int64_t d : dims) {
    write_varint_field(tensor, tensor_field::kDims, static_cast<uint64_t>(d));
  }
  write_varint_field(tensor, tensor_field::kDataType, data_type);
  write_bytes_field(tensor, tensor_field::kRawData, raw_data);
  return tensor;
}

std::string encode_tensor_attribute(const std::string& name, const std::string& tensor) {
  std::string attr;
  write_bytes_field(attr, attribute_field::kName, name);
  write_bytes_field(attr, attribute_field::kT, tensor);
  write_varint_field(attr, attribute_field::kType, attribute_type::kTensor);
  return attr;
}

/// Appends one encoded `GraphProto.node` field to `out`.
void emit_node(std::string& out, const std::string& op_type, const std::string& name,
               const std::vector<std::string>& inputs, const std::vector<std::string>& outputs,
               const std::vector<std::string>& attributes = {}) {
  std::string node;
  for (const auto& in : inputs) {
    write_bytes_field(node, node_field::kInput, in);
  }
  for (const auto& o : outputs) {
    write_bytes_field(node, node_field::kOutput, o);
  }
  write_bytes_field(node, node_field::kName, name);
  write_bytes_field(node, node_field::kOpType, op_type);
  for (const auto& attr : attributes) {
    write_bytes_field(node, node_field::kAttribute, attr);
  }
  write_bytes_field(out, graph_field::kNode, node);
}

bool is_default_domain(const std::string& domain) { return domain.empty() || domain == "ai.onnx"; }

/// Element types of the named values of a graph, as far as they can be
/// determined without running full ONNX type inference.
using TypeMap = std::unordered_map<std::string, uint64_t>;

void add_value_info_type(TypeMap& types, std::string_view value_info) {
  std::string name;
  std::optional<uint64_t> elem_type;
  Reader reader(value_info);
  while (!reader.done()) {
    const Field field = reader.next();
    if (field.number == value_info_field::kName) {
      name = std::string(field.payload);
    } else if (field.number == value_info_field::kType && field.wire_type == kLengthDelimited) {
      Reader type_reader(field.payload);
      while (!type_reader.done()) {
        const Field type = type_reader.next();
        if (type.number != type_field::kTensorType || type.wire_type != kLengthDelimited) {
          continue;
        }
        Reader tensor_type_reader(type.payload);
        while (!tensor_type_reader.done()) {
          const Field tensor_type = tensor_type_reader.next();
          if (tensor_type.number == type_field::kElemType && tensor_type.wire_type == kVarint) {
            elem_type = tensor_type.value;
          }
        }
      }
    }
  }
  if (!name.empty() && elem_type) {
    types.emplace(name, *elem_type);
  }
}

TypeMap collect_types(std::string_view graph) {
  TypeMap types;
  Reader reader(graph);
  while (!reader.done()) {
    const Field field = reader.next();
    if (field.wire_type != kLengthDelimited) {
      continue;
    }
    switch (field.number) {
      case graph_field::kInitializer: {
        std::string name;
        Reader tensor_reader(field.payload);
        while (!tensor_reader.done()) {
          const Field tensor = tensor_reader.next();
          if (tensor.number == tensor_field::kName) {
            name = std::string(tensor.payload);
          }
        }
        const auto type = tensor_data_type(field.payload);
        if (!name.empty() && type) {
          types.emplace(name, *type);
        }
        break;
      }
      case graph_field::kInput:
      case graph_field::kOutput:
      case graph_field::kValueInfo:
        add_value_info_type(types, field.payload);
        break;
      case graph_field::kNode: {
        const Node node = parse_node(field.payload);
        if (node.outputs.empty() || !is_default_domain(node.domain)) {
          break;
        }
        if (node.op_type == "Cast" && node.cast_to) {
          types.emplace(node.outputs[0], *node.cast_to);
        } else if (node.op_type == "Constant" && node.value_type) {
          types.emplace(node.outputs[0], *node.value_type);
        }
        break;
      }
      default:
        break;
    }
  }
  return types;
}

/// Element type of a LayerNormalization node. Its inputs X, Scale and B all
/// share one type, so any of them identifies it.
std::optional<uint64_t> layer_norm_type(const Node& node, const TypeMap& types) {
  for (const auto& input : node.inputs) {
    const auto it = types.find(input);
    if (it != types.end()) {
      return it->second;
    }
  }
  return std::nullopt;
}

/// Returns true if the node can be decomposed without knowing the input rank
/// and without dropping anything the graph depends on.
bool can_decompose(const Node& node, std::optional<uint64_t> type) {
  const int64_t axis = node.axis.value_or(-1);
  if (axis >= 0 || node.has_other_attributes || !type) {
    return false;
  }
  if (node.inputs.size() < 2 || node.inputs[0].empty() || node.inputs[1].empty() || node.outputs.empty() ||
      node.outputs[0].empty()) {
    return false;
  }
  // The optional Mean / InvStdDev outputs are not produced by the
  // decomposition, so only nodes that leave them unused are supported.
  for (size_t i = 1; i < node.outputs.size(); ++i) {
    if (!node.outputs[i].empty()) {
      return false;
    }
  }
  return true;
}

/// Appends the primitive-op equivalent of a LayerNormalization node:
///   mean = ReduceMean(x), d = x - mean, var = ReduceMean(d * d)
///   y    = d / Sqrt(var + epsilon) * scale (+ bias)
/// Like the ONNX op (with its default stash_type), the computation is done
/// in float32; inputs of other types are cast to float32 and the result is
/// cast back.
void emit_decomposition(std::string& out, const Node& node, uint64_t type, int index, int64_t opset) {
  const std::string base =
      (node.name.empty() ? node.outputs[0] : node.name) + kDecomposedNodeTag + std::to_string(index) + "/";
  const auto name = [&base](const char* suffix) { return base + suffix; };

  const bool needs_cast = type != tensor_type::kFloat;
  const auto as_float = [&](const std::string& input, const char* suffix) {
    if (!needs_cast) {
      return input;
    }
    emit_node(out, "Cast", name(suffix), {input}, {name(suffix)},
              {encode_int_attribute("to", static_cast<int64_t>(tensor_type::kFloat))});
    return name(suffix);
  };

  // Normalize over all dimensions from `axis` to the last one.
  const int64_t axis = node.axis.value_or(-1);
  std::vector<int64_t> axes;
  for (int64_t a = axis; a < 0; ++a) {
    axes.push_back(a);
  }

  // Since opset 18 ReduceMean takes `axes` as an input instead of an attribute.
  std::vector<std::string> reduce_attributes{encode_ints_attribute("axes", axes)};
  std::vector<std::string> reduce_extra_inputs;
  if (opset >= 18) {
    std::string raw(axes.size() * sizeof(int64_t), '\0');
    std::memcpy(raw.data(), axes.data(), raw.size());
    emit_node(out, "Constant", name("axes"), {}, {name("axes")},
              {encode_tensor_attribute("value",
                                       encode_tensor(tensor_type::kInt64, {static_cast<int64_t>(axes.size())}, raw))});
    reduce_attributes.clear();
    reduce_extra_inputs.push_back(name("axes"));
  }
  const auto reduce_inputs = [&reduce_extra_inputs](const std::string& data) {
    std::vector<std::string> inputs{data};
    inputs.insert(inputs.end(), reduce_extra_inputs.begin(), reduce_extra_inputs.end());
    return inputs;
  };

  const float epsilon = node.epsilon.value_or(1e-5f);
  std::string eps_raw(sizeof(float), '\0');
  std::memcpy(eps_raw.data(), &epsilon, sizeof(float));

  const std::string x = as_float(node.inputs[0], "x_float");
  const std::string scale = as_float(node.inputs[1], "scale_float");
  const bool has_bias = node.inputs.size() > 2 && !node.inputs[2].empty();
  const std::string bias = has_bias ? as_float(node.inputs[2], "bias_float") : std::string();
  const std::string& y = node.outputs[0];

  emit_node(out, "ReduceMean", name("mean"), reduce_inputs(x), {name("mean")}, reduce_attributes);
  emit_node(out, "Sub", name("centered"), {x, name("mean")}, {name("centered")});
  emit_node(out, "Mul", name("squared"), {name("centered"), name("centered")}, {name("squared")});
  emit_node(out, "ReduceMean", name("variance"), reduce_inputs(name("squared")), {name("variance")}, reduce_attributes);
  emit_node(out, "Constant", name("epsilon"), {}, {name("epsilon")},
            {encode_tensor_attribute("value", encode_tensor(tensor_type::kFloat, {}, eps_raw))});
  emit_node(out, "Add", name("variance_eps"), {name("variance"), name("epsilon")}, {name("variance_eps")});
  emit_node(out, "Sqrt", name("std"), {name("variance_eps")}, {name("std")});
  emit_node(out, "Div", name("normalized"), {name("centered"), name("std")}, {name("normalized")});

  const std::string result = needs_cast ? name("result") : y;
  if (has_bias) {
    emit_node(out, "Mul", name("scaled"), {name("normalized"), scale}, {name("scaled")});
    emit_node(out, "Add", name("biased"), {name("scaled"), bias}, {result});
  } else {
    emit_node(out, "Mul", name("scaled"), {name("normalized"), scale}, {result});
  }
  if (needs_cast) {
    emit_node(out, "Cast", name("y_cast"), {result}, {y}, {encode_int_attribute("to", static_cast<int64_t>(type))});
  }
}

int64_t default_domain_opset(std::string_view model) {
  int64_t opset = 0;
  Reader reader(model);
  while (!reader.done()) {
    const Field field = reader.next();
    if (field.number != model_field::kOpsetImport || field.wire_type != kLengthDelimited) {
      continue;
    }
    std::string domain;
    int64_t version = 0;
    Reader opset_reader(field.payload);
    while (!opset_reader.done()) {
      const Field opset_field = opset_reader.next();
      if (opset_field.number == opset_field::kDomain) {
        domain = std::string(opset_field.payload);
      } else if (opset_field.number == opset_field::kVersion) {
        version = static_cast<int64_t>(opset_field.value);
      }
    }
    if (is_default_domain(domain)) {
      opset = version;
    }
  }
  return opset;
}

}  // namespace

bool is_precision_sensitive_layer(const std::string& layer_name) {
  if (layer_name.find(kDecomposedNodeTag) == std::string::npos) {
    return false;
  }
  for (const char* suffix : {"/squared", "/variance", "/variance_eps", "/std"}) {
    const size_t len = std::strlen(suffix);
    if (layer_name.size() >= len && layer_name.compare(layer_name.size() - len, len, suffix) == 0) {
      return true;
    }
  }
  return false;
}

LayerNormDecompositionResult decompose_layer_normalization(const std::string& serialized_model) {
  LayerNormDecompositionResult result;
  const int64_t opset = default_domain_opset(serialized_model);

  std::string model;
  model.reserve(serialized_model.size() + serialized_model.size() / 100);
  Reader model_reader(serialized_model);
  while (!model_reader.done()) {
    const Field model_field = model_reader.next();
    if (model_field.number != model_field::kGraph || model_field.wire_type != kLengthDelimited) {
      model.append(model_field.raw);
      continue;
    }

    const TypeMap types = collect_types(model_field.payload);
    std::string graph;
    graph.reserve(model_field.payload.size() + model_field.payload.size() / 100);
    Reader graph_reader(model_field.payload);
    while (!graph_reader.done()) {
      const Field graph_field = graph_reader.next();
      if (graph_field.number != graph_field::kNode || graph_field.wire_type != kLengthDelimited) {
        graph.append(graph_field.raw);
        continue;
      }
      const Node node = parse_node(graph_field.payload);
      if (node.op_type != "LayerNormalization" || !is_default_domain(node.domain)) {
        graph.append(graph_field.raw);
        continue;
      }
      const auto type = layer_norm_type(node, types);
      if (!can_decompose(node, type)) {
        graph.append(graph_field.raw);
        ++result.num_skipped;
      } else {
        emit_decomposition(graph, node, *type, result.num_decomposed, opset);
        ++result.num_decomposed;
      }
    }
    write_bytes_field(model, model_field::kGraph, graph);
  }

  result.model = result.num_decomposed > 0 ? std::move(model) : serialized_model;
  return result;
}

}  // namespace bitbots_vision::onnx_rewrite
