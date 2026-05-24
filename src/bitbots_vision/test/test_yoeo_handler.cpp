#include <gtest/gtest.h>
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

#include <algorithm>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Execution provider availability tests
// ---------------------------------------------------------------------------

// These tests verify that the ONNX Runtime built-in provider list API works
// correctly and that our priority-based registration logic is sound — without
// actually loading a model.

TEST(GetAvailableProviders, ReturnsList) {
  const auto providers = Ort::GetAvailableProviders();
  EXPECT_FALSE(providers.empty()) << "ONNX Runtime must always have at least CPUExecutionProvider";
}

TEST(GetAvailableProviders, AlwaysHasCPU) {
  const auto providers = Ort::GetAvailableProviders();
  bool has_cpu = std::find(providers.begin(), providers.end(), "CPUExecutionProvider") != providers.end();
  EXPECT_TRUE(has_cpu) << "CPUExecutionProvider must always be available";
}

TEST(GetAvailableProviders, KnownProviderNames) {
  // Providers must use the canonical ORT names, not shortened versions.
  // This test documents the expected names so any future rename is caught early.
  const auto providers = Ort::GetAvailableProviders();

  // Every name in the list must be non-empty
  for (const auto& ep : providers) {
    EXPECT_FALSE(ep.empty());
  }

  // If CUDA is present it must use the canonical full name
  bool has_cuda = std::find(providers.begin(), providers.end(), "CUDAExecutionProvider") != providers.end();
  bool has_cuda_short = std::find(providers.begin(), providers.end(), "CUDA") != providers.end();
  EXPECT_FALSE(has_cuda_short) << "ORT should report 'CUDAExecutionProvider', not 'CUDA'";
  (void)has_cuda;  // presence is optional — just validate the name if present
}

// ---------------------------------------------------------------------------
// Provider priority selection helper (mirrors yoeo_handler.cpp logic)
// ---------------------------------------------------------------------------

static std::vector<std::string> select_providers(const std::vector<std::string>& available,
                                                 const std::vector<std::string>& priority) {
  std::vector<std::string> selected;
  for (const auto& ep : priority) {
    if (std::find(available.begin(), available.end(), ep) != available.end()) {
      selected.push_back(ep);
    }
  }
  return selected;
}

TEST(ProviderPrioritySelection, OrderPreserved) {
  const std::vector<std::string> available = {"CUDAExecutionProvider", "CPUExecutionProvider",
                                              "TensorrtExecutionProvider"};
  const std::vector<std::string> priority = {"TensorrtExecutionProvider", "CUDAExecutionProvider",
                                             "CPUExecutionProvider"};

  auto selected = select_providers(available, priority);

  ASSERT_EQ(selected.size(), 3u);
  EXPECT_EQ(selected[0], "TensorrtExecutionProvider");
  EXPECT_EQ(selected[1], "CUDAExecutionProvider");
  EXPECT_EQ(selected[2], "CPUExecutionProvider");
}

TEST(ProviderPrioritySelection, UnavailableProviderSkipped) {
  const std::vector<std::string> available = {"CPUExecutionProvider"};
  const std::vector<std::string> priority = {"TensorrtExecutionProvider", "CUDAExecutionProvider",
                                             "CPUExecutionProvider"};

  auto selected = select_providers(available, priority);

  ASSERT_EQ(selected.size(), 1u);
  EXPECT_EQ(selected[0], "CPUExecutionProvider");
}

TEST(ProviderPrioritySelection, AllAvailable) {
  const std::vector<std::string> available = {
      "TensorrtExecutionProvider",
      "CUDAExecutionProvider",
      "WebGPUExecutionProvider",
      "CPUExecutionProvider",
  };
  const std::vector<std::string> priority = {
      "TensorrtExecutionProvider",
      "CUDAExecutionProvider",
      "WebGPUExecutionProvider",
      "CPUExecutionProvider",
  };

  auto selected = select_providers(available, priority);

  EXPECT_EQ(selected, priority);
}

TEST(ProviderPrioritySelection, NoneAvailableFromPriority) {
  // ORT build has only CPU, but priority asks for GPU providers that don't exist here
  const std::vector<std::string> available = {"CPUExecutionProvider"};
  const std::vector<std::string> priority = {"TensorrtExecutionProvider", "CUDAExecutionProvider"};

  auto selected = select_providers(available, priority);
  EXPECT_TRUE(selected.empty());
}

TEST(ProviderPrioritySelection, RealOrtBuildAlwaysSelectsCPU) {
  // Integration check: running against the actual ORT build
  const auto available = Ort::GetAvailableProviders();
  const std::vector<std::string> priority = {
      "TensorrtExecutionProvider",
      "CUDAExecutionProvider",
      "WebGPUExecutionProvider",
      "CPUExecutionProvider",
  };

  auto selected = select_providers(available, priority);

  EXPECT_FALSE(selected.empty()) << "At least CPUExecutionProvider should be selected";
  EXPECT_EQ(selected.back(), "CPUExecutionProvider");
}
