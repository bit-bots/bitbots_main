#pragma once

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <bitbots_vision/candidate.hpp>
#include <bitbots_vision/model_config.hpp>
#include <bitbots_vision/rfdetr_config.hpp>
#include <bitbots_vision/rfdetr_handler_interface.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace bitbots_vision {

/// TensorRT-native RF-DETR inference handler (no ONNX Runtime involved),
/// preferred over `RfdetrHandler` whenever TensorRT was found at build time
/// -- see CMakeLists.txt and vision_node.cpp. Targets JetPack 5.1.5
/// (TensorRT 8.5.2 / CUDA 11.4), but only relies on TensorRT APIs that have
/// been stable since TensorRT 8.0.
///
/// On first construction for a given model directory, a TensorRT engine is
/// built from `<model_dir>/onnx/rfdetr.onnx` and cached to
/// `<model_dir>/onnx/rfdetr.trt` (this can take several minutes on a
/// Jetson); subsequent runs load the cached engine directly (a few
/// seconds). The cache is rebuilt automatically if the .onnx file is newer
/// than the cached engine. A TensorRT engine is tied to the exact GPU +
/// TensorRT version it was built on -- delete the cached .trt file to force
/// a rebuild after a TensorRT/driver upgrade or a GPU swap.
///
/// FP16 is enabled automatically when the platform supports it
/// (`IBuilder::platformHasFastFp16()`, true on all JetPack 5.1.5
/// Xavier/Orin targets); TensorRT decides per-layer whether FP16 or FP32 is
/// used, so this is safe even for layers that don't support FP16. The same
/// fp32 ONNX export used by `RfdetrHandler` is used as the build source --
/// there is no need for a separately-exported fp16 ONNX file here.
class TensorRtHandler : public RfdetrHandlerInterface {
 public:
  TensorRtHandler(const std::string& model_path, const ModelConfig& model_config, const RfdetrConfig& cfg,
                  const rclcpp::Logger& logger);
  ~TensorRtHandler() override;

  TensorRtHandler(const TensorRtHandler&) = delete;
  TensorRtHandler& operator=(const TensorRtHandler&) = delete;

  void reconfigure(const RfdetrConfig& cfg) override;
  void set_image(const cv::Mat& bgr_image) override;
  void predict() override;
  std::vector<Candidate> get_detection_candidates_for(const std::string& class_name) override;
  cv::Mat get_line_mask() override;
  const std::vector<std::string>& detection_class_names() const override;

 private:
  // Generic deleter for TensorRT interface objects: since TensorRT 8.0 these
  // support plain `delete` (the older `->destroy()` pattern is deprecated).
  struct TrtDeleter {
    template <typename T>
    void operator()(T* obj) const {
      delete obj;
    }
  };

  // Forwards TensorRT's internal log messages to the ROS logger.
  class TrtLogger : public nvinfer1::ILogger {
   public:
    explicit TrtLogger(rclcpp::Logger ros_logger) : ros_logger_(std::move(ros_logger)) {}

    void log(Severity severity, const char* msg) noexcept override {
      switch (severity) {
        case Severity::kINTERNAL_ERROR:
        case Severity::kERROR:
          RCLCPP_ERROR(ros_logger_, "[TensorRT] %s", msg);
          break;
        case Severity::kWARNING:
          RCLCPP_WARN(ros_logger_, "[TensorRT] %s", msg);
          break;
        case Severity::kINFO:
          RCLCPP_INFO(ros_logger_, "[TensorRT] %s", msg);
          break;
        case Severity::kVERBOSE:
        default:
          break;  // too noisy to forward by default
      }
    }

   private:
    rclcpp::Logger ros_logger_;
  };

  // ----- TensorRT objects -----
  TrtLogger trt_logger_;
  std::unique_ptr<nvinfer1::IRuntime, TrtDeleter> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, TrtDeleter> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, TrtDeleter> context_;
  cudaStream_t stream_{nullptr};

  // ----- Bindings (resolved by name at load time; -1 if not present, only
  // valid for line_mask_binding_idx_) -----
  int input_binding_idx_{-1};
  int dets_binding_idx_{-1};
  int labels_binding_idx_{-1};
  int line_mask_binding_idx_{-1};

  // ----- Device buffers -----
  void* d_input_{nullptr};
  void* d_dets_{nullptr};
  void* d_labels_{nullptr};
  void* d_line_mask_{nullptr};

  // ----- Model metadata -----
  int net_h_{0};
  int net_w_{0};
  int64_t num_dets_{0};
  int64_t num_classes_{0};
  int line_mask_h_{0};
  int line_mask_w_{0};
  std::vector<std::string> det_class_names_;

  // ----- Runtime state -----
  RfdetrConfig cfg_;
  rclcpp::Logger logger_;

  std::vector<float> input_data_;
  int orig_h_{0};
  int orig_w_{0};

  bool prediction_is_fresh_{true};

  std::unordered_map<std::string, std::vector<Candidate>> det_results_;
  cv::Mat line_mask_result_;

  // ----- Helpers -----
  void build_or_load_engine(const std::string& model_path);
  void allocate_buffers();
  void preprocess(const cv::Mat& bgr_image);
  void run_inference();
};

}  // namespace bitbots_vision
