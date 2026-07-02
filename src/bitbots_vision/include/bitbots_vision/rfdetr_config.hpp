#pragma once

#include <string>
#include <unordered_map>

namespace bitbots_vision {

/// Configuration shared by every RF-DETR inference backend (ONNX Runtime,
/// TensorRT, ...), so callers can build one config and hand it to whichever
/// backend was compiled in.
struct RfdetrConfig {
  /// Per-class-name softmax-confidence thresholds (e.g. "ball" -> 0.98).
  /// Classes not present here fall back to `default_conf_threshold`.
  std::unordered_map<std::string, float> class_conf_thresholds;
  float default_conf_threshold{0.99f};
  /// Post-sigmoid probability threshold for the line-segmentation head.
  float line_mask_threshold{0.5f};
};

}  // namespace bitbots_vision
