#pragma once

#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

#include <bitbots_vision/candidate.hpp>
#include <bitbots_vision/model_config.hpp>
#include <bitbots_vision/rfdetr_config.hpp>
#include <bitbots_vision/rfdetr_handler_interface.hpp>
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
/// This is the portable, always-built backend. When TensorRT was found at
/// build time, `TensorRtHandler` (native nvinfer1, no ONNX Runtime) is
/// preferred instead -- see CMakeLists.txt and vision_node.cpp.
///
/// The model must expose:
///   - one input named "input"  shape [1, 3, H, W]
///   - outputs read BY NAME (not position, since the output count/order can
///     vary between model exports):
///       "dets"       shape [1, N, 4]              normalized (cx, cy, w, h)
///       "labels"     shape [1, N, num_classes]     raw class logits
///       "line_mask"  shape [1, 1, Hd, Wd]          raw line-segmentation logits (optional)
///
/// RF-DETR is NMS-free (a fixed number of object queries N, e.g. 300), so
/// unlike YOEO there is no NMS step. Field lines are read directly from the
/// model's own learned segmentation head (if present) rather than a
/// classical color-based heuristic.
///
/// FP16-weight model exports keep the same float32 input/output tensor
/// contract as the fp32 export (only internal weights differ), so no
/// handler code needs to change to use one -- just point `model_path` at a
/// model directory containing the fp16-weight onnx file.
class RfdetrHandler : public RfdetrHandlerInterface {
 public:
  RfdetrHandler(const std::string& model_path, const ModelConfig& model_config, const RfdetrConfig& cfg,
                const rclcpp::Logger& logger);

  /// Update thresholds without reloading the model.
  void reconfigure(const RfdetrConfig& cfg) override;

  /// Set the image to be processed by the next call to predict().
  /// The image must be in BGR8 format (as delivered by cv_bridge).
  void set_image(const cv::Mat& bgr_image) override;

  /// Run the network on the current image (no-op if already up to date).
  void predict() override;

  std::vector<Candidate> get_detection_candidates_for(const std::string& class_name) override;

  /// Line-segmentation mask (CV_8UC1, 0/255) at original image resolution.
  /// Empty if the loaded model has no "line_mask" output.
  cv::Mat get_line_mask() override;

  const std::vector<std::string>& detection_class_names() const override;

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

  // Indices into output_names_/the Run() result vector, resolved by name at
  // load time (-1 if not present, only valid for line_mask_idx_).
  int dets_idx_{-1};
  int labels_idx_{-1};
  int line_mask_idx_{-1};

  // ----- Runtime state -----
  RfdetrConfig cfg_;
  rclcpp::Logger logger_;

  std::vector<float> input_data_;
  int orig_h_{0};
  int orig_w_{0};

  bool prediction_is_fresh_{true};

  // Cached results
  std::unordered_map<std::string, std::vector<Candidate>> det_results_;
  cv::Mat line_mask_result_;

  // ----- Helpers -----
  void init_session(const std::string& model_path);
  void preprocess(const cv::Mat& bgr_image);
  void run_inference();
};

}  // namespace bitbots_vision
