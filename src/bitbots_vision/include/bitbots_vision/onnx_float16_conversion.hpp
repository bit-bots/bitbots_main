#pragma once

#include <string>

/// Converts the float16 parts of a serialized ONNX model to float32.
///
/// Some execution providers cannot run float16 graphs on every device, e.g.
/// ONNX Runtime's WebGPU provider on GPUs without shader-f16 support. Our
/// RF-DETR exports compute in float16 internally, so on such devices the
/// session falls back to the CPU. Converting the model to float32 lets these
/// providers run it on the GPU instead, at the cost of more memory traffic.
///
/// Converted are: float16 initializers and tensor attributes (e.g. Constant
/// values), the element types of graph inputs / outputs / value infos, and
/// the target type of Cast and of the ops with a `dtype` attribute. This is
/// applied recursively to subgraphs. The rewrite works directly on the
/// protobuf wire format, so no ONNX / protobuf library is required.
namespace bitbots_vision::onnx_rewrite {

struct Float16ConversionResult {
  /// Serialized `onnx.ModelProto` without float16 tensors or types.
  /// Identical to the input if the model contains no float16 parts.
  std::string model;
  /// Number of float16 tensors (initializers and tensor attributes) converted.
  int num_tensors{0};
  /// Number of float16 element types (value infos) and op target types converted.
  int num_types{0};
};

/// Throws std::runtime_error if the model cannot be parsed or stores a float16
/// tensor in an external data file.
Float16ConversionResult convert_float16_to_float32(const std::string& serialized_model);

}  // namespace bitbots_vision::onnx_rewrite
