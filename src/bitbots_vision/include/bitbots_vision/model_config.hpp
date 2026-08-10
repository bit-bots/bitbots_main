#pragma once

#include <string>
#include <vector>

namespace bitbots_vision {

/// Configuration loaded from the model's `model_config.yaml`.
class ModelConfig {
 public:
  ModelConfig() = default;

  /// Load from `<model_dir>/model_config.yaml`.  Throws on error, including if
  /// `detection.classes` is missing or empty.
  static ModelConfig load_from(const std::string& model_dir);

  const std::vector<std::string>& detection_classes() const { return detection_classes_; }

  /// Indices of all classes whose name contains "robot".
  std::vector<int> robot_class_ids() const;

 private:
  std::vector<std::string> detection_classes_;
};

}  // namespace bitbots_vision
