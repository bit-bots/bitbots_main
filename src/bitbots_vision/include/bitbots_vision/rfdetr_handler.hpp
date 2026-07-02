#pragma once

#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

#include <bitbots_vision/candidate.hpp>
#include <bitbots_vision/model_config.hpp>
#include <bitbots_vision/rfdetr_processing.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <rclcpp/logger.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace bitbots_vision {

/// ONNX-Runtime-based RF-DETR inference handler.
///
/// Execution providers are tried in priority order:
///   TensorRT → CUDA → WebGPU → CPU (always available as fallback)
///
/// The model must expose:
///   - one input named "input"  shape [1, 3, H, W]
///   - two outputs, read POSITIONALLY (not by name, matching the reference
///     Python implementation's `session.run(None, ...)`):
///       [0] dets    shape [1, N, 4]              normalized (cx, cy, w, h)
///       [1] logits  shape [1, N, num_classes]     raw class logits
///
/// RF-DETR is NMS-free (a fixed number of object queries N, e.g. 300), so
/// unlike YOEO there is no NMS step and no segmentation output.
class RfdetrHandler {
 public:
  struct Config {
    /// Per-class-name softmax-confidence thresholds (e.g. "ball" -> 0.98).
    /// Classes not present here fall back to `default_conf_threshold`.
    std::unordered_map<std::string, float> class_conf_thresholds;
    float default_conf_threshold{0.99f};
  };

  RfdetrHandler(const std::string& model_path, const ModelConfig& model_config, const Config& cfg,
                const rclcpp::Logger& logger);

  /// Update thresholds without reloading the model.
  void reconfigure(const Config& cfg);

  /// Set the image to be processed by the next call to predict().
  /// The image must be in BGR8 format (as delivered by cv_bridge).
  void set_image(const cv::Mat& bgr_image);

  /// Run the network on the current image (no-op if already up to date).
  void predict();

  std::vector<Candidate> get_detection_candidates_for(const std::string& class_name);

  const std::vector<std::string>& detection_class_names() const;

 private:
  // ----- ONNX Runtime objects -----
  Ort::Env env_;
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;

  // ----- Model metadata -----
  std::vector<int64_t> input_shape_;  // [1, 3, H_net, W_net]
  std::string input_name_;
  std::vector<std::string> output_names_;
  std::vector<std::string> det_class_names_;

  // ----- Runtime state -----
  Config cfg_;
  rclcpp::Logger logger_;

  std::vector<float> input_data_;
  int orig_h_{0};
  int orig_w_{0};

  bool prediction_is_fresh_{true};

  // Cached results
  std::unordered_map<std::string, std::vector<Candidate>> det_results_;

  // ----- Helpers -----
  void init_session(const std::string& model_path);
  void preprocess(const cv::Mat& bgr_image);
  void run_inference();
};

}  // namespace bitbots_vision
