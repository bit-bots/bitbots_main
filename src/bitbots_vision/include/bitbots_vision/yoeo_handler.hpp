#pragma once

#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

#include <memory>
#include <opencv2/core.hpp>
#include <rclcpp/logger.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "bitbots_vision/candidate.hpp"
#include "bitbots_vision/model_config.hpp"
#include "bitbots_vision/yoeo_processing.hpp"

namespace bitbots_vision {

/// ONNX-Runtime-based YOEO inference handler.
///
/// Execution providers are tried in priority order:
///   TensorRT → CUDA → WebGPU → CPU (always available as fallback)
///
/// The model must expose:
///   - one input named "InputLayer"  shape [1, 3, H, W]
///   - two outputs:
///       [0] detections     shape [1, N, 5+num_classes]  (x_c, y_c, w, h, obj_conf, class_probs…)
///       [1] segmentation   shape [1, H_seg, W_seg]      (argmax class index per pixel)
class YoeoHandler {
 public:
  struct Config {
    float conf_threshold{0.5f};
    float nms_threshold{0.4f};
  };

  YoeoHandler(const std::string& model_path, const ModelConfig& model_config, const Config& cfg,
              const rclcpp::Logger& logger);

  /// Update thresholds without reloading the model.
  void reconfigure(const Config& cfg);

  /// Set the image to be processed by the next call to predict().
  /// The image must be in BGR8 format (as delivered by cv_bridge).
  void set_image(const cv::Mat& bgr_image);

  /// Run the network on the current image (no-op if already up to date).
  void predict();

  std::vector<Candidate> get_detection_candidates_for(const std::string& class_name);
  cv::Mat get_segmentation_mask_for(const std::string& class_name);

  const std::vector<std::string>& detection_class_names() const;
  const std::vector<std::string>& segmentation_class_names() const;

 private:
  // ----- ONNX Runtime objects -----
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;

  // ----- Model metadata -----
  std::vector<int64_t> input_shape_;  // [1, 3, H_net, W_net]
  std::string input_name_;
  std::vector<std::string> output_names_;
  int num_det_classes_{0};
  std::vector<int> robot_class_ids_;
  std::vector<std::string> det_class_names_;
  std::vector<std::string> seg_class_names_;

  // ----- Runtime state -----
  Config cfg_;
  rclcpp::Logger logger_;

  cv::Mat current_image_rgb_;  // float32, CHW layout, stored as [C, H, W] in flat vector
  std::vector<float> input_data_;

  bool prediction_is_fresh_{true};

  // Preprocessing metadata for postprocessing
  processing::PreprocessInfo preprocess_info_;

  // Cached results
  std::unordered_map<std::string, std::vector<Candidate>> det_results_;
  std::unordered_map<std::string, cv::Mat> seg_results_;  // CV_8UC1 binary masks

  // ----- Helpers -----
  void init_session(const std::string& model_path);
  void preprocess(const cv::Mat& bgr_image);
  void run_inference();
  void postprocess_detections(const float* det_data, const std::vector<int64_t>& shape);
  void postprocess_segmentation(const uint8_t* seg_data, const std::vector<int64_t>& shape);
};

}  // namespace bitbots_vision
