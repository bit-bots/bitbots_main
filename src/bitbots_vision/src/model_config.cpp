#include "bitbots_vision/model_config.hpp"

#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace bitbots_vision {

ModelConfig ModelConfig::load_from(const std::string & model_dir)
{
  const std::string path = model_dir + "/model_config.yaml";
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("model_config.yaml not found at: " + path);
  }

  YAML::Node root = YAML::LoadFile(path);

  ModelConfig cfg;

  for (const auto & name : root["detection"]["classes"]) {
    cfg.detection_classes_.push_back(name.as<std::string>());
  }

  for (const auto & name : root["segmentation"]["classes"]) {
    cfg.segmentation_classes_.push_back(name.as<std::string>());
  }

  if (root["detection"]["team_colors"]) {
    cfg.team_colors_provided_ = root["detection"]["team_colors"].as<bool>();
  }

  return cfg;
}

std::vector<int> ModelConfig::robot_class_ids() const
{
  std::vector<int> ids;
  for (int i = 0; i < static_cast<int>(detection_classes_.size()); ++i) {
    if (detection_classes_[i].find("robot") != std::string::npos) {
      ids.push_back(i);
    }
  }
  return ids;
}

}  // namespace bitbots_vision
