#include <bitbots_vision/onnx_float16_conversion.hpp>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "onnx-ml.pb.h"  // generated from proto/onnx-ml.proto

namespace bitbots_vision::onnx_rewrite {

namespace {

namespace pb = bitbots_vision::onnx;

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

void convert_graph(pb::GraphProto& graph, Counts& counts);

void convert_tensor(pb::TensorProto& tensor, Counts& counts) {
  if (tensor.data_type() != pb::TensorProto::FLOAT16) {
    return;
  }
  if (tensor.data_location() == pb::TensorProto::EXTERNAL) {
    throw std::runtime_error("ONNX float16 conversion: tensors in external data files are not supported");
  }

  // float16 values are stored either in raw_data or as their bit pattern in int32_data
  std::vector<uint16_t> halves;
  if (tensor.has_raw_data()) {
    const std::string& raw = tensor.raw_data();
    halves.resize(raw.size() / sizeof(uint16_t));
    std::memcpy(halves.data(), raw.data(), halves.size() * sizeof(uint16_t));
  } else {
    halves.reserve(static_cast<size_t>(tensor.int32_data_size()));
    for (const int32_t bits : tensor.int32_data()) {
      halves.push_back(static_cast<uint16_t>(bits));
    }
  }

  std::string raw(halves.size() * sizeof(float), '\0');
  for (size_t i = 0; i < halves.size(); ++i) {
    const float value = half_to_float(halves[i]);
    std::memcpy(raw.data() + i * sizeof(float), &value, sizeof(float));
  }
  tensor.clear_int32_data();
  tensor.set_raw_data(std::move(raw));
  tensor.set_data_type(pb::TensorProto::FLOAT);
  ++counts.tensors;
}

void convert_elem_type(int32_t& elem_type, Counts& counts) {
  if (elem_type == pb::TensorProto::FLOAT16) {
    elem_type = pb::TensorProto::FLOAT;
    ++counts.types;
  }
}

void convert_type(pb::TypeProto& type, Counts& counts) {
  if (type.has_tensor_type()) {
    int32_t elem_type = type.tensor_type().elem_type();
    convert_elem_type(elem_type, counts);
    type.mutable_tensor_type()->set_elem_type(elem_type);
  }
  if (type.has_sparse_tensor_type()) {
    int32_t elem_type = type.sparse_tensor_type().elem_type();
    convert_elem_type(elem_type, counts);
    type.mutable_sparse_tensor_type()->set_elem_type(elem_type);
  }
  if (type.has_sequence_type() && type.sequence_type().has_elem_type()) {
    convert_type(*type.mutable_sequence_type()->mutable_elem_type(), counts);
  }
  if (type.has_optional_type() && type.optional_type().has_elem_type()) {
    convert_type(*type.mutable_optional_type()->mutable_elem_type(), counts);
  }
  if (type.has_map_type() && type.map_type().has_value_type()) {
    convert_type(*type.mutable_map_type()->mutable_value_type(), counts);
  }
}

void convert_value_info(pb::ValueInfoProto& value_info, Counts& counts) {
  if (value_info.has_type()) {
    convert_type(*value_info.mutable_type(), counts);
  }
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

void convert_node(pb::NodeProto& node, Counts& counts) {
  for (pb::AttributeProto& attribute : *node.mutable_attribute()) {
    if (attribute.has_i() && attribute.i() == pb::TensorProto::FLOAT16 &&
        is_type_attribute(node.op_type(), attribute.name())) {
      attribute.set_i(pb::TensorProto::FLOAT);
      ++counts.types;
    }
    if (attribute.has_t()) {
      convert_tensor(*attribute.mutable_t(), counts);
    }
    for (pb::TensorProto& tensor : *attribute.mutable_tensors()) {
      convert_tensor(tensor, counts);
    }
    if (attribute.has_g()) {
      convert_graph(*attribute.mutable_g(), counts);
    }
    for (pb::GraphProto& graph : *attribute.mutable_graphs()) {
      convert_graph(graph, counts);
    }
  }
}

void convert_graph(pb::GraphProto& graph, Counts& counts) {
  for (pb::NodeProto& node : *graph.mutable_node()) {
    convert_node(node, counts);
  }
  for (pb::TensorProto& tensor : *graph.mutable_initializer()) {
    convert_tensor(tensor, counts);
  }
  for (pb::SparseTensorProto& sparse : *graph.mutable_sparse_initializer()) {
    if (sparse.has_values()) {
      convert_tensor(*sparse.mutable_values(), counts);
    }
  }
  for (auto* value_infos : {graph.mutable_input(), graph.mutable_output(), graph.mutable_value_info()}) {
    for (pb::ValueInfoProto& value_info : *value_infos) {
      convert_value_info(value_info, counts);
    }
  }
}

}  // namespace

Float16ConversionResult convert_float16_to_float32(const std::string& serialized_model) {
  pb::ModelProto model;
  if (!model.ParseFromString(serialized_model)) {
    throw std::runtime_error("ONNX float16 conversion: the model could not be parsed");
  }

  Counts counts;
  if (model.has_graph()) {
    convert_graph(*model.mutable_graph(), counts);
  }
  // Model-local functions can contain float16 nodes as well
  for (pb::FunctionProto& function : *model.mutable_functions()) {
    for (pb::NodeProto& node : *function.mutable_node()) {
      convert_node(node, counts);
    }
  }

  Float16ConversionResult result;
  result.num_tensors = counts.tensors;
  result.num_types = counts.types;
  if (counts.tensors > 0 || counts.types > 0) {
    if (!model.SerializeToString(&result.model)) {
      throw std::runtime_error("ONNX float16 conversion: the converted model could not be serialized");
    }
  } else {
    result.model = serialized_model;
  }
  return result;
}

}  // namespace bitbots_vision::onnx_rewrite
