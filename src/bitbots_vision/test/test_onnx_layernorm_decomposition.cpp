#include <gtest/gtest.h>
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

#include <bitbots_vision/onnx_layernorm_decomposition.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

using bitbots_vision::onnx_rewrite::decompose_layer_normalization;

// ===========================================================================
// Minimal ONNX model builder (protobuf wire format, field numbers from
// onnx.proto), so the tests do not need an ONNX / protobuf library.
// ===========================================================================

namespace {

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

void field_float(std::string& out, uint32_t number, float f) {
  varint(out, (static_cast<uint64_t>(number) << 3) | 5);
  char bytes[4];
  std::memcpy(bytes, &f, 4);
  out.append(bytes, 4);
}

constexpr uint64_t kFloat = 1;
constexpr uint64_t kFloat16 = 10;

// IEEE 754 binary16 conversion (round-to-nearest-even, normal range only --
// sufficient for the small test values used here).
uint16_t to_half(float f) {
  uint32_t bits;
  std::memcpy(&bits, &f, 4);
  const uint32_t sign = (bits >> 16) & 0x8000;
  const int32_t exponent = static_cast<int32_t>((bits >> 23) & 0xFF) - 127 + 15;
  uint32_t mantissa = bits & 0x7FFFFF;
  if (exponent <= 0) {
    return static_cast<uint16_t>(sign);
  }
  uint32_t half = sign | (static_cast<uint32_t>(exponent) << 10) | (mantissa >> 13);
  const uint32_t rest = mantissa & 0x1FFF;
  if (rest > 0x1000 || (rest == 0x1000 && (half & 1))) {
    ++half;
  }
  return static_cast<uint16_t>(half);
}

float from_half(uint16_t h) {
  const uint32_t sign = static_cast<uint32_t>(h & 0x8000) << 16;
  const uint32_t exponent = (h >> 10) & 0x1F;
  const uint32_t mantissa = h & 0x3FF;
  uint32_t bits = sign;
  if (exponent != 0) {
    bits |= ((exponent - 15 + 127) << 23) | (mantissa << 13);
  }
  float f;
  std::memcpy(&f, &bits, 4);
  return f;
}

std::string encode_values(const std::vector<float>& data, uint64_t elem_type) {
  if (elem_type == kFloat16) {
    std::vector<uint16_t> half(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
      half[i] = to_half(data[i]);
    }
    return std::string(reinterpret_cast<const char*>(half.data()), half.size() * sizeof(uint16_t));
  }
  return std::string(reinterpret_cast<const char*>(data.data()), data.size() * sizeof(float));
}

std::string tensor(const std::string& name, const std::vector<int64_t>& dims, const std::vector<float>& data,
                   uint64_t elem_type) {
  std::string t;
  for (const auto d : dims) {
    field_varint(t, 1, static_cast<uint64_t>(d));
  }
  field_varint(t, 2, elem_type);
  field_bytes(t, 8, name);
  field_bytes(t, 9, encode_values(data, elem_type));
  return t;
}

std::string value_info(const std::string& name, const std::vector<int64_t>& dims, uint64_t elem_type) {
  std::string shape;
  for (const auto d : dims) {
    std::string dim;
    field_varint(dim, 1, static_cast<uint64_t>(d));
    field_bytes(shape, 1, dim);
  }
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

struct LayerNormSpec {
  std::optional<int64_t> axis;
  std::optional<float> epsilon;
  bool with_bias{true};
  bool with_mean_output{false};
  uint64_t elem_type{kFloat};
};

constexpr int64_t kDim0 = 2;
constexpr int64_t kDim1 = 3;
constexpr int64_t kDim2 = 5;

/// Model: Y = LayerNormalization(X, scale, bias) followed by an Identity, so
/// the LN node sits between other nodes like in a real network.
std::string build_model(const LayerNormSpec& spec) {
  const int64_t axis = spec.axis.value_or(-1);
  std::vector<int64_t> norm_dims;
  const std::vector<int64_t> x_dims{kDim0, kDim1, kDim2};
  for (int64_t a = static_cast<int64_t>(x_dims.size()) + axis; a < static_cast<int64_t>(x_dims.size()); ++a) {
    norm_dims.push_back(x_dims[static_cast<size_t>(a)]);
  }
  int64_t norm_size = 1;
  for (const auto d : norm_dims) {
    norm_size *= d;
  }
  std::vector<float> scale(static_cast<size_t>(norm_size));
  std::vector<float> bias(static_cast<size_t>(norm_size));
  for (int64_t i = 0; i < norm_size; ++i) {
    scale[static_cast<size_t>(i)] = 0.5f + 0.1f * static_cast<float>(i);
    bias[static_cast<size_t>(i)] = -0.2f + 0.05f * static_cast<float>(i);
  }

  std::string ln;
  field_bytes(ln, 1, "X");
  field_bytes(ln, 1, "scale");
  if (spec.with_bias) {
    field_bytes(ln, 1, "bias");
  }
  field_bytes(ln, 2, "ln_out");
  if (spec.with_mean_output) {
    field_bytes(ln, 2, "Mean");
  }
  field_bytes(ln, 3, "encoder/norm1/LayerNormalization");
  field_bytes(ln, 4, "LayerNormalization");
  if (spec.axis) {
    std::string attr;
    field_bytes(attr, 1, "axis");
    field_varint(attr, 3, static_cast<uint64_t>(*spec.axis));
    field_varint(attr, 20, 2);  // INT
    field_bytes(ln, 5, attr);
  }
  if (spec.epsilon) {
    std::string attr;
    field_bytes(attr, 1, "epsilon");
    field_float(attr, 2, *spec.epsilon);
    field_varint(attr, 20, 1);  // FLOAT
    field_bytes(ln, 5, attr);
  }

  std::string identity;
  field_bytes(identity, 1, "ln_out");
  field_bytes(identity, 2, "Y");
  field_bytes(identity, 3, "identity");
  field_bytes(identity, 4, "Identity");

  std::string graph;
  field_bytes(graph, 1, ln);
  field_bytes(graph, 1, identity);
  field_bytes(graph, 2, "test_graph");
  field_bytes(graph, 5, tensor("scale", norm_dims, scale, spec.elem_type));
  if (spec.with_bias) {
    field_bytes(graph, 5, tensor("bias", norm_dims, bias, spec.elem_type));
  }
  field_bytes(graph, 11, value_info("X", x_dims, spec.elem_type));
  field_bytes(graph, 12, value_info("Y", x_dims, spec.elem_type));

  std::string opset;
  field_bytes(opset, 1, "");
  field_varint(opset, 2, 17);

  std::string model;
  field_varint(model, 1, 8);  // ir_version
  field_bytes(model, 2, "bitbots_vision_test");
  field_bytes(model, 7, graph);
  field_bytes(model, 8, opset);
  return model;
}

std::vector<float> test_input() {
  std::vector<float> x(static_cast<size_t>(kDim0 * kDim1 * kDim2));
  for (size_t i = 0; i < x.size(); ++i) {
    x[i] = std::sin(static_cast<float>(i) * 0.7f) * 3.0f + static_cast<float>(i % 4);
  }
  return x;
}

std::vector<float> run(const std::string& model, uint64_t elem_type = kFloat) {
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test_onnx_layernorm_decomposition");
  Ort::SessionOptions options;
  options.SetIntraOpNumThreads(1);
  Ort::Session session(env, model.data(), model.size(), options);

  const std::vector<float> x = test_input();
  std::string x_bytes = encode_values(x, elem_type);
  const std::vector<int64_t> shape{kDim0, kDim1, kDim2};
  auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input = Ort::Value::CreateTensor(
      mem, x_bytes.data(), x_bytes.size(), shape.data(), shape.size(),
      elem_type == kFloat16 ? ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16 : ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT);
  const char* in_names[] = {"X"};
  const char* out_names[] = {"Y"};
  auto outputs = session.Run(Ort::RunOptions{nullptr}, in_names, &input, 1, out_names, 1);
  std::vector<float> y(x.size());
  if (elem_type == kFloat16) {
    const auto* data = static_cast<const uint16_t*>(outputs[0].GetTensorRawData());
    for (size_t i = 0; i < y.size(); ++i) {
      y[i] = from_half(data[i]);
    }
  } else {
    const float* data = outputs[0].GetTensorData<float>();
    y.assign(data, data + y.size());
  }
  return y;
}

void expect_equivalent(const LayerNormSpec& spec) {
  const std::string original = build_model(spec);
  const auto result = decompose_layer_normalization(original);
  ASSERT_EQ(result.num_decomposed, 1);
  EXPECT_EQ(result.num_skipped, 0);
  // No node may keep op_type (field 4, length 18) "LayerNormalization".
  EXPECT_EQ(result.model.find(std::string("\x22\x12LayerNormalization")), std::string::npos);

  const auto expected = run(original);
  const auto actual = run(result.model);
  ASSERT_EQ(expected.size(), actual.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(expected[i], actual[i], 1e-4f) << "at index " << i;
  }
}

}  // namespace

// ===========================================================================
// Tests
// ===========================================================================

TEST(DecomposeLayerNormalization, LastAxisWithBiasMatchesNativeOp) { expect_equivalent({-1, 1e-6f, true, false}); }

TEST(DecomposeLayerNormalization, DefaultAttributesMatchNativeOp) {
  expect_equivalent({std::nullopt, std::nullopt, true, false});
}

TEST(DecomposeLayerNormalization, WithoutBiasMatchesNativeOp) { expect_equivalent({-1, 1e-5f, false, false}); }

TEST(DecomposeLayerNormalization, MultipleTrailingAxesMatchNativeOp) { expect_equivalent({-2, 1e-5f, true, false}); }

TEST(DecomposeLayerNormalization, Float16ComputesInFloat32AndMatchesFloatReference) {
  LayerNormSpec spec{-1, 1e-6f, true, false};
  const auto reference = run(build_model(spec));
  spec.elem_type = kFloat16;
  const auto result = decompose_layer_normalization(build_model(spec));
  ASSERT_EQ(result.num_decomposed, 1);
  const auto actual = run(result.model, kFloat16);
  for (size_t i = 0; i < reference.size(); ++i) {
    // Only the fp16 rounding of inputs and output remains.
    EXPECT_NEAR(reference[i], actual[i], 1e-2f) << "at index " << i;
  }
}

TEST(DecomposeLayerNormalization, DecomposedNodesCarryTag) {
  const auto result = decompose_layer_normalization(build_model({-1, 1e-6f, true, false}));
  EXPECT_NE(result.model.find(std::string("encoder/norm1/LayerNormalization") +
                              bitbots_vision::onnx_rewrite::kDecomposedNodeTag),
            std::string::npos);
}

TEST(DecomposeLayerNormalization, PositiveAxisIsLeftUnchanged) {
  const std::string original = build_model({2, 1e-5f, true, false});
  const auto result = decompose_layer_normalization(original);
  EXPECT_EQ(result.num_decomposed, 0);
  EXPECT_EQ(result.num_skipped, 1);
  EXPECT_EQ(result.model, original);
}

TEST(DecomposeLayerNormalization, UsedMeanOutputIsLeftUnchanged) {
  const std::string original = build_model({-1, 1e-5f, true, true});
  const auto result = decompose_layer_normalization(original);
  EXPECT_EQ(result.num_decomposed, 0);
  EXPECT_EQ(result.num_skipped, 1);
  EXPECT_EQ(result.model, original);
}

TEST(DecomposeLayerNormalization, MalformedModelThrows) {
  EXPECT_THROW(decompose_layer_normalization(std::string("\x3a\xff\xff\xff\x0f", 5)), std::runtime_error);
}
