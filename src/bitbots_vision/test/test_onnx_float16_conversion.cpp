#include <gtest/gtest.h>
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

#include <bitbots_vision/onnx_float16_conversion.hpp>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using bitbots_vision::onnx_rewrite::convert_float16_to_float32;

// ===========================================================================
// Minimal ONNX model builder (protobuf wire format, field numbers from
// onnx.proto), so the tests do not need an ONNX / protobuf library.
// ===========================================================================

namespace {

constexpr uint64_t kFloat = 1;
constexpr uint64_t kFloat16 = 10;
constexpr int64_t kSize = 6;

void varint(std::string& out, uint64_t v) {
  while (v >= 0x80) {
    out.push_back(static_cast<char>((v & 0x7F) | 0x80));
    v >>= 7;
  }
  out.push_back(static_cast<char>(v));
}

void field_varint(std::string& out, uint32_t number, uint64_t v) {
  varint(out, static_cast<uint64_t>(number) << 3);
  varint(out, v);
}

void field_bytes(std::string& out, uint32_t number, const std::string& bytes) {
  varint(out, (static_cast<uint64_t>(number) << 3) | 2);
  varint(out, bytes.size());
  out += bytes;
}

// float -> IEEE 754 binary16 for exactly representable test values
uint16_t to_half(float f) {
  uint32_t bits;
  std::memcpy(&bits, &f, 4);
  const uint32_t sign = (bits >> 16) & 0x8000;
  const int32_t exponent = static_cast<int32_t>((bits >> 23) & 0xFF) - 127 + 15;
  if (f == 0.0f) {
    return static_cast<uint16_t>(sign);
  }
  return static_cast<uint16_t>(sign | (static_cast<uint32_t>(exponent) << 10) | ((bits & 0x7FFFFF) >> 13));
}

std::vector<float> test_values() { return {0.5f, -1.25f, 2.0f, 0.0f, 3.75f, -0.125f}; }

/// float16 tensor with the data in raw_data or, alternatively, as bit patterns in int32_data.
std::string half_tensor(const std::string& name, const std::vector<float>& values, bool use_int32_data) {
  std::string t;
  field_varint(t, 1, static_cast<uint64_t>(values.size()));  // dims
  field_varint(t, 2, kFloat16);
  field_bytes(t, 8, name);
  if (use_int32_data) {
    std::string packed;
    for (const float v : values) {
      varint(packed, to_half(v));
    }
    field_bytes(t, 5, packed);
  } else {
    std::string raw;
    for (const float v : values) {
      const uint16_t h = to_half(v);
      raw.append(reinterpret_cast<const char*>(&h), sizeof(h));
    }
    field_bytes(t, 9, raw);
  }
  return t;
}

std::string value_info(const std::string& name, uint64_t elem_type) {
  std::string dim;
  field_varint(dim, 1, kSize);
  std::string shape;
  field_bytes(shape, 1, dim);
  std::string tensor_type;
  field_varint(tensor_type, 1, elem_type);
  field_bytes(tensor_type, 2, shape);
  std::string type;
  field_bytes(type, 1, tensor_type);
  std::string vi;
  field_bytes(vi, 1, name);
  field_bytes(vi, 2, type);
  return vi;
}

std::string node(const std::string& op, const std::vector<std::string>& inputs, const std::string& output,
                 const std::vector<std::string>& attributes = {}) {
  std::string n;
  for (const auto& in : inputs) {
    field_bytes(n, 1, in);
  }
  field_bytes(n, 2, output);
  field_bytes(n, 3, output + "_node");
  field_bytes(n, 4, op);
  for (const auto& a : attributes) {
    field_bytes(n, 5, a);
  }
  return n;
}

std::string int_attribute(const std::string& name, uint64_t value) {
  std::string a;
  field_bytes(a, 1, name);
  field_varint(a, 3, value);
  field_varint(a, 20, 2);  // INT
  return a;
}

std::string tensor_attribute(const std::string& name, const std::string& tensor) {
  std::string a;
  field_bytes(a, 1, name);
  field_bytes(a, 5, tensor);
  field_varint(a, 20, 4);  // TENSOR
  return a;
}

/// Y = Cast<float>(Cast<float16>(X) + bias + Constant), computed in float16 like our exports.
std::string build_mixed_precision_model() {
  std::string graph;
  field_bytes(graph, 1, node("Cast", {"X"}, "x_half", {int_attribute("to", kFloat16)}));
  field_bytes(graph, 1,
              node("Constant", {}, "offset", {tensor_attribute("value", half_tensor("", test_values(), true))}));
  field_bytes(graph, 1, node("Add", {"x_half", "bias"}, "sum"));
  field_bytes(graph, 1, node("Add", {"sum", "offset"}, "sum2"));
  field_bytes(graph, 1, node("Cast", {"sum2"}, "Y", {int_attribute("to", kFloat)}));
  field_bytes(graph, 2, "mixed_precision");
  field_bytes(graph, 5, half_tensor("bias", test_values(), false));
  field_bytes(graph, 11, value_info("X", kFloat));
  field_bytes(graph, 12, value_info("Y", kFloat));
  field_bytes(graph, 13, value_info("sum", kFloat16));

  std::string opset;
  field_bytes(opset, 1, "");
  field_varint(opset, 2, 17);
  std::string model;
  field_varint(model, 1, 8);  // ir_version
  field_bytes(model, 7, graph);
  field_bytes(model, 8, opset);
  return model;
}

std::vector<float> run(const std::string& model) {
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test_onnx_float16_conversion");
  Ort::SessionOptions options;
  Ort::Session session(env, model.data(), model.size(), options);
  std::vector<float> x{1.0f, 2.0f, -3.0f, 0.25f, 0.5f, 8.0f};
  const std::vector<int64_t> shape{kSize};
  auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input = Ort::Value::CreateTensor<float>(mem, x.data(), x.size(), shape.data(), shape.size());
  const char* in_names[] = {"X"};
  const char* out_names[] = {"Y"};
  auto outputs = session.Run(Ort::RunOptions{nullptr}, in_names, &input, 1, out_names, 1);
  const float* y = outputs[0].GetTensorData<float>();
  return {y, y + kSize};
}

}  // namespace

// ===========================================================================
// Tests
// ===========================================================================

TEST(ConvertFloat16ToFloat32, ConvertsTensorsAndTypes) {
  const auto result = convert_float16_to_float32(build_mixed_precision_model());
  EXPECT_EQ(result.num_tensors, 2);  // bias initializer + Constant value
  EXPECT_EQ(result.num_types, 2);    // Cast "to" + value info of "sum"
}

TEST(ConvertFloat16ToFloat32, ConvertedModelMatchesOriginal) {
  const std::string original = build_mixed_precision_model();
  const auto result = convert_float16_to_float32(original);
  const auto expected = run(original);
  const auto actual = run(result.model);
  ASSERT_EQ(expected.size(), actual.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    // All values are exactly representable in float16, so both must agree exactly.
    EXPECT_FLOAT_EQ(expected[i], actual[i]) << "at index " << i;
  }
}

TEST(ConvertFloat16ToFloat32, IsIdempotent) {
  const auto once = convert_float16_to_float32(build_mixed_precision_model());
  const auto twice = convert_float16_to_float32(once.model);
  EXPECT_EQ(twice.num_tensors, 0);
  EXPECT_EQ(twice.num_types, 0);
  EXPECT_EQ(twice.model, once.model);
}

TEST(ConvertFloat16ToFloat32, MalformedModelThrows) {
  EXPECT_THROW(convert_float16_to_float32(std::string("\x3a\xff\xff\xff\x0f", 5)), std::runtime_error);
}
