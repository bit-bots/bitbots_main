#include <bitbots_vision/onnx_float16_conversion.hpp>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "onnx_wire_format.hpp"

namespace bitbots_vision::onnx_rewrite {

namespace {

using onnx_wire::Field;
using onnx_wire::kLengthDelimited;
using onnx_wire::kVarint;
using onnx_wire::Reader;
using onnx_wire::write_bytes_field;
using onnx_wire::write_varint_field;

// Field numbers from onnx.proto
namespace model_field {
constexpr uint32_t kGraph = 7;
}  // namespace model_field
namespace graph_field {
constexpr uint32_t kNode = 1;
constexpr uint32_t kInitializer = 5;
constexpr uint32_t kInput = 11;
constexpr uint32_t kOutput = 12;
constexpr uint32_t kValueInfo = 13;
}  // namespace graph_field
namespace node_field {
constexpr uint32_t kOpType = 4;
constexpr uint32_t kAttribute = 5;
}  // namespace node_field
namespace attribute_field {
constexpr uint32_t kName = 1;
constexpr uint32_t kI = 3;
constexpr uint32_t kT = 5;
constexpr uint32_t kG = 6;
constexpr uint32_t kTensors = 10;
constexpr uint32_t kGraphs = 11;
}  // namespace attribute_field
namespace tensor_field {
constexpr uint32_t kDataType = 2;
constexpr uint32_t kInt32Data = 5;
constexpr uint32_t kRawData = 9;
constexpr uint32_t kDataLocation = 14;
}  // namespace tensor_field
namespace value_info_field {
constexpr uint32_t kType = 2;
}  // namespace value_info_field
namespace type_field {
constexpr uint32_t kTensorType = 1;
constexpr uint32_t kSequenceType = 4;
constexpr uint32_t kSparseTensorType = 8;
constexpr uint32_t kOptionalType = 9;
constexpr uint32_t kElemType = 1;  // in Tensor, SparseTensor, Sequence and Optional
}  // namespace type_field

constexpr uint64_t kFloat = 1;
constexpr uint64_t kFloat16 = 10;
constexpr uint64_t kDataLocationExternal = 1;

float half_to_float(uint16_t h) {
  const uint32_t sign = static_cast<uint32_t>(h & 0x8000) << 16;
  const uint32_t exponent = (h >> 10) & 0x1F;
  uint32_t mantissa = h & 0x3FF;
  uint32_t bits;
  if (exponent == 0x1F) {
    bits = sign | 0x7F800000 | (mantissa << 13);  // inf / nan
  } else if (exponent != 0) {
    bits = sign | ((exponent + 127 - 15) << 23) | (mantissa << 13);
  } else if (mantissa == 0) {
    bits = sign;  // +-0
  } else {
    // Subnormal half: normalize it for the float representation
    int e = -1;
    do {
      ++e;
      mantissa <<= 1;
    } while ((mantissa & 0x400) == 0);
    bits = sign | (static_cast<uint32_t>(127 - 15 - e) << 23) | ((mantissa & 0x3FF) << 13);
  }
  float f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

struct Counts {
  int tensors{0};
  int types{0};
};

std::string convert_graph(std::string_view graph, Counts& counts);

/// Returns the converted TensorProto, or the input if it is not float16.
std::string convert_tensor(std::string_view tensor, Counts& counts) {
  std::optional<uint64_t> data_type;
  bool external = false;
  {
    Reader reader(tensor);
    while (!reader.done()) {
      const Field f = reader.next();
      if (f.number == tensor_field::kDataType && f.wire_type == kVarint) {
        data_type = f.value;
      } else if (f.number == tensor_field::kDataLocation && f.wire_type == kVarint) {
        external = f.value == kDataLocationExternal;
      }
    }
  }
  if (data_type != kFloat16) {
    return std::string(tensor);
  }
  if (external) {
    throw std::runtime_error("ONNX float16 conversion: tensors in external data files are not supported");
  }

  std::vector<uint16_t> halves;
  std::string out;
  Reader reader(tensor);
  while (!reader.done()) {
    const Field f = reader.next();
    if (f.number == tensor_field::kDataType) {
      continue;  // rewritten below
    } else if (f.number == tensor_field::kRawData && f.wire_type == kLengthDelimited) {
      const size_t n = f.payload.size() / sizeof(uint16_t);
      for (size_t i = 0; i < n; ++i) {
        uint16_t h;
        std::memcpy(&h, f.payload.data() + i * sizeof(uint16_t), sizeof(h));
        halves.push_back(h);
      }
    } else if (f.number == tensor_field::kInt32Data) {
      // float16 values may also be stored as the bit pattern in int32_data,
      // either packed or as single varints.
      if (f.wire_type == kLengthDelimited) {
        Reader packed(f.payload);
        while (!packed.done()) {
          halves.push_back(static_cast<uint16_t>(packed.read_varint()));
        }
      } else {
        halves.push_back(static_cast<uint16_t>(f.value));
      }
    } else {
      out.append(f.raw);
    }
  }

  std::string raw(halves.size() * sizeof(float), '\0');
  for (size_t i = 0; i < halves.size(); ++i) {
    const float value = half_to_float(halves[i]);
    std::memcpy(raw.data() + i * sizeof(float), &value, sizeof(float));
  }
  write_varint_field(out, tensor_field::kDataType, kFloat);
  write_bytes_field(out, tensor_field::kRawData, raw);
  ++counts.tensors;
  return out;
}

/// Converts the element type of a TypeProto (recursing into sequences and optionals).
std::string convert_type(std::string_view type, Counts& counts) {
  std::string out;
  Reader reader(type);
  while (!reader.done()) {
    const Field f = reader.next();
    const bool is_tensor = f.number == type_field::kTensorType || f.number == type_field::kSparseTensorType;
    const bool is_container = f.number == type_field::kSequenceType || f.number == type_field::kOptionalType;
    if (f.wire_type != kLengthDelimited || (!is_tensor && !is_container)) {
      out.append(f.raw);
      continue;
    }
    std::string inner;
    Reader inner_reader(f.payload);
    while (!inner_reader.done()) {
      const Field g = inner_reader.next();
      if (is_tensor && g.number == type_field::kElemType && g.wire_type == kVarint && g.value == kFloat16) {
        write_varint_field(inner, type_field::kElemType, kFloat);
        ++counts.types;
      } else if (is_container && g.number == type_field::kElemType && g.wire_type == kLengthDelimited) {
        write_bytes_field(inner, type_field::kElemType, convert_type(g.payload, counts));
      } else {
        inner.append(g.raw);
      }
    }
    write_bytes_field(out, f.number, inner);
  }
  return out;
}

std::string convert_value_info(std::string_view value_info, Counts& counts) {
  std::string out;
  Reader reader(value_info);
  while (!reader.done()) {
    const Field f = reader.next();
    if (f.number == value_info_field::kType && f.wire_type == kLengthDelimited) {
      write_bytes_field(out, f.number, convert_type(f.payload, counts));
    } else {
      out.append(f.raw);
    }
  }
  return out;
}

/// Whether the integer attribute `name` of an op of type `op_type` holds a target element type.
bool is_type_attribute(std::string_view op_type, std::string_view name) {
  if (op_type == "Cast") {
    return name == "to";
  }
  if (name != "dtype") {
    return false;
  }
  for (const char* op : {"EyeLike", "RandomNormal", "RandomNormalLike", "RandomUniform", "RandomUniformLike",
                         "Bernoulli", "Multinomial"}) {
    if (op_type == op) {
      return true;
    }
  }
  return false;
}

std::string convert_attribute(std::string_view attribute, std::string_view op_type, Counts& counts) {
  std::string name;
  {
    Reader reader(attribute);
    while (!reader.done()) {
      const Field f = reader.next();
      if (f.number == attribute_field::kName) {
        name = std::string(f.payload);
      }
    }
  }
  const bool type_attribute = is_type_attribute(op_type, name);

  std::string out;
  Reader reader(attribute);
  while (!reader.done()) {
    const Field f = reader.next();
    if (f.number == attribute_field::kI && f.wire_type == kVarint && type_attribute && f.value == kFloat16) {
      write_varint_field(out, f.number, kFloat);
      ++counts.types;
    } else if ((f.number == attribute_field::kT || f.number == attribute_field::kTensors) &&
               f.wire_type == kLengthDelimited) {
      write_bytes_field(out, f.number, convert_tensor(f.payload, counts));
    } else if ((f.number == attribute_field::kG || f.number == attribute_field::kGraphs) &&
               f.wire_type == kLengthDelimited) {
      write_bytes_field(out, f.number, convert_graph(f.payload, counts));
    } else {
      out.append(f.raw);
    }
  }
  return out;
}

std::string convert_node(std::string_view node, Counts& counts) {
  std::string op_type;
  {
    Reader reader(node);
    while (!reader.done()) {
      const Field f = reader.next();
      if (f.number == node_field::kOpType) {
        op_type = std::string(f.payload);
      }
    }
  }
  std::string out;
  Reader reader(node);
  while (!reader.done()) {
    const Field f = reader.next();
    if (f.number == node_field::kAttribute && f.wire_type == kLengthDelimited) {
      write_bytes_field(out, f.number, convert_attribute(f.payload, op_type, counts));
    } else {
      out.append(f.raw);
    }
  }
  return out;
}

std::string convert_graph(std::string_view graph, Counts& counts) {
  std::string out;
  out.reserve(graph.size() + graph.size() / 2);
  Reader reader(graph);
  while (!reader.done()) {
    const Field f = reader.next();
    if (f.wire_type != kLengthDelimited) {
      out.append(f.raw);
      continue;
    }
    switch (f.number) {
      case graph_field::kNode:
        write_bytes_field(out, f.number, convert_node(f.payload, counts));
        break;
      case graph_field::kInitializer:
        write_bytes_field(out, f.number, convert_tensor(f.payload, counts));
        break;
      case graph_field::kInput:
      case graph_field::kOutput:
      case graph_field::kValueInfo:
        write_bytes_field(out, f.number, convert_value_info(f.payload, counts));
        break;
      default:
        out.append(f.raw);
        break;
    }
  }
  return out;
}

}  // namespace

Float16ConversionResult convert_float16_to_float32(const std::string& serialized_model) {
  Counts counts;
  std::string model;
  model.reserve(serialized_model.size() * 2);
  Reader reader(serialized_model);
  while (!reader.done()) {
    const Field f = reader.next();
    if (f.number == model_field::kGraph && f.wire_type == kLengthDelimited) {
      write_bytes_field(model, f.number, convert_graph(f.payload, counts));
    } else {
      model.append(f.raw);
    }
  }

  Float16ConversionResult result;
  result.num_tensors = counts.tensors;
  result.num_types = counts.types;
  result.model = (counts.tensors > 0 || counts.types > 0) ? std::move(model) : serialized_model;
  return result;
}

}  // namespace bitbots_vision::onnx_rewrite
