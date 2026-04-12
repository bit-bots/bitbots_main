#pragma once

#include <string>
#include <vector>

namespace bitbots_vision {

/// Configuration loaded from the model's `model_config.yaml`.
class ModelConfig {
 public:
  ModelConfig() = default;

  /// Load from `<model_dir>/model_config.yaml`.  Throws on error.
  static ModelConfig load_from(const std::string& model_dir);

  const std::vector<std::string>& detection_classes() const { return detection_classes_; }
  const std::vector<std::string>& segmentation_classes() const { return segmentation_classes_; }
  bool team_colors_provided() const { return team_colors_provided_; }

  /// Indices of all classes whose name contains "robot".
  std::vector<int> robot_class_ids() const;

 private:
  std::vector<std::string> detection_classes_;
  std::vector<std::string> segmentation_classes_;
  bool team_colors_provided_{false};
};

}  // namespace bitbots_vision
