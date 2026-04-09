#include "bitbots_vision/yoeo_handler.hpp"

#include <filesystem>
#include <stdexcept>

#include <rclcpp/logging.hpp>

#include "bitbots_vision/yoeo_processing.hpp"

namespace bitbots_vision {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

YoeoHandler::YoeoHandler(
  const std::string & model_path,
  const ModelConfig & model_config,
  const Config & cfg,
  const rclcpp::Logger & logger)
: env_(ORT_LOGGING_LEVEL_WARNING, "bitbots_vision"),
  cfg_(cfg),
  logger_(logger)
{
  det_class_names_ = model_config.detection_classes();
  seg_class_names_ = model_config.segmentation_classes();
  robot_class_ids_ = model_config.robot_class_ids();
  num_det_classes_ = static_cast<int>(det_class_names_.size());

  init_session(model_path);
}

// ---------------------------------------------------------------------------
// init_session – builds the ONNX session with provider fallback chain
// ---------------------------------------------------------------------------

void YoeoHandler::init_session(const std::string & model_path)
{
  const std::string onnx_file = model_path + "/onnx/yoeo.onnx";
  if (!std::filesystem::exists(onnx_file)) {
    throw std::runtime_error("ONNX model file not found: " + onnx_file);
  }

  session_options_.SetIntraOpNumThreads(1);
  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  // Register execution providers in priority order.
  // Ort::GetAvailableProviders() returns providers compiled into this ORT build.
  // CUDA and TensorRT require typed registration APIs; other providers use the generic API.
  const auto available = Ort::GetAvailableProviders();
  auto has_ep = [&](const std::string & name) {
    return std::find(available.begin(), available.end(), name) != available.end();
  };

  // TRT and CUDA use typed registration APIs and dynamically load their provider shared
  // libraries at registration time. Wrap in try/catch so a missing system library
  // (e.g. libcublasLt.so) causes graceful fallback rather than a crash.
  if (has_ep("TensorrtExecutionProvider")) {
    try {
      OrtTensorRTProviderOptions trt_opts{};
      session_options_.AppendExecutionProvider_TensorRT(trt_opts);
      RCLCPP_INFO(logger_, "ONNX Runtime: registered TensorrtExecutionProvider");
    } catch (const Ort::Exception & e) {
      RCLCPP_WARN(logger_, "TensorrtExecutionProvider unavailable: %s", e.what());
    }
  }

  if (has_ep("CUDAExecutionProvider")) {
    try {
      OrtCUDAProviderOptions cuda_opts{};
      session_options_.AppendExecutionProvider_CUDA(cuda_opts);
      RCLCPP_INFO(logger_, "ONNX Runtime: registered CUDAExecutionProvider");
    } catch (const Ort::Exception & e) {
      RCLCPP_WARN(logger_, "CUDAExecutionProvider unavailable: %s", e.what());
    }
  }

  if (has_ep("WebGPUExecutionProvider")) {
    try {
      session_options_.AppendExecutionProvider("WebGPU", {});
      RCLCPP_INFO(logger_, "ONNX Runtime: registered WebGPUExecutionProvider");
    } catch (const Ort::Exception & e) {
      RCLCPP_WARN(logger_, "WebGPUExecutionProvider unavailable: %s", e.what());
    }
  }

  // CPU is always the implicit fallback — no explicit registration needed.

  RCLCPP_INFO(logger_, "Loading ONNX model: %s", onnx_file.c_str());
  session_ = std::make_unique<Ort::Session>(env_, onnx_file.c_str(), session_options_);

  // ---- Introspect model I/O ----
  Ort::AllocatorWithDefaultOptions allocator;

  // Input
  auto in_name_ptr = session_->GetInputNameAllocated(0, allocator);
  input_name_ = std::string(in_name_ptr.get());
  input_shape_ = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  // Expected shape: [1, 3, H_net, W_net]
  // Fix dynamic batch dim; spatial dims must be positive for YOEO models.
  if (input_shape_.size() == 4) {
    if (input_shape_[0] < 0) {
      input_shape_[0] = 1;
    }
    if (input_shape_[2] <= 0 || input_shape_[3] <= 0) {
      RCLCPP_WARN(logger_, "Dynamic spatial dims in ONNX model – falling back to 416×416");
      input_shape_[2] = 416;
      input_shape_[3] = 416;
    }
  }

  // Outputs
  const size_t num_outputs = session_->GetOutputCount();
  for (size_t i = 0; i < num_outputs; ++i) {
    auto out_name_ptr = session_->GetOutputNameAllocated(i, allocator);
    output_names_.emplace_back(out_name_ptr.get());
  }

  RCLCPP_INFO(
    logger_,
    "Model loaded – input '%s' [1, 3, %lld, %lld], %zu outputs",
    input_name_.c_str(),
    static_cast<long long>(input_shape_[2]),
    static_cast<long long>(input_shape_[3]),
    num_outputs);
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void YoeoHandler::reconfigure(const Config & cfg)
{
  cfg_ = cfg;
}

void YoeoHandler::set_image(const cv::Mat & bgr_image)
{
  det_results_.clear();
  seg_results_.clear();
  preprocess(bgr_image);
  prediction_is_fresh_ = false;
}

void YoeoHandler::predict()
{
  if (prediction_is_fresh_) {
    return;
  }
  run_inference();
  prediction_is_fresh_ = true;
}

std::vector<Candidate> YoeoHandler::get_detection_candidates_for(const std::string & class_name)
{
  predict();
  auto it = det_results_.find(class_name);
  return (it != det_results_.end()) ? it->second : std::vector<Candidate>{};
}

cv::Mat YoeoHandler::get_segmentation_mask_for(const std::string & class_name)
{
  predict();
  auto it = seg_results_.find(class_name);
  return (it != seg_results_.end()) ? it->second : cv::Mat{};
}

const std::vector<std::string> & YoeoHandler::detection_class_names() const
{
  return det_class_names_;
}

const std::vector<std::string> & YoeoHandler::segmentation_class_names() const
{
  return seg_class_names_;
}

// ---------------------------------------------------------------------------
// Preprocessing  (BGR uint8 → RGB float32, padded to square, CHW)
// ---------------------------------------------------------------------------

void YoeoHandler::preprocess(const cv::Mat & bgr_image)
{
  const int net_h = static_cast<int>(input_shape_[2]);
  const int net_w = static_cast<int>(input_shape_[3]);
  input_data_ = processing::preprocess_image(bgr_image, net_h, net_w, preprocess_info_);
  input_shape_[0] = 1;  // batch dimension is always 1
}

// ---------------------------------------------------------------------------
// Inference
// ---------------------------------------------------------------------------

void YoeoHandler::run_inference()
{
  auto memory_info =
    Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    memory_info,
    input_data_.data(), input_data_.size(),
    input_shape_.data(), input_shape_.size());

  const char * input_name_cstr = input_name_.c_str();
  std::vector<const char *> out_name_cstrs;
  for (const auto & n : output_names_) {
    out_name_cstrs.push_back(n.c_str());
  }

  auto outputs = session_->Run(
    Ort::RunOptions{nullptr},
    &input_name_cstr, &input_tensor, 1,
    out_name_cstrs.data(), out_name_cstrs.size());

  if (outputs.size() < 2) {
    RCLCPP_ERROR(logger_, "Expected at least 2 outputs from YOEO model, got %zu", outputs.size());
    return;
  }

  {
    auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    postprocess_detections(outputs[0].GetTensorMutableData<float>(), shape);
  }
  {
    auto shape = outputs[1].GetTensorTypeAndShapeInfo().GetShape();
    postprocess_segmentation(outputs[1].GetTensorMutableData<float>(), shape);
  }
}

// ---------------------------------------------------------------------------
// Detection post-processing
// ---------------------------------------------------------------------------

void YoeoHandler::postprocess_detections(
  const float * det_data,
  const std::vector<int64_t> & shape)
{
  if (shape.size() < 3) {
    return;
  }
  det_results_ = processing::postprocess_detections(
    det_data, shape[1], shape[2],
    det_class_names_, robot_class_ids_,
    cfg_.conf_threshold, cfg_.nms_threshold,
    preprocess_info_);
}

// ---------------------------------------------------------------------------
// Segmentation post-processing
// ---------------------------------------------------------------------------

void YoeoHandler::postprocess_segmentation(
  const float * seg_data,
  const std::vector<int64_t> & shape)
{
  if (shape.size() < 3) {
    return;
  }
  seg_results_ = processing::postprocess_segmentation(
    seg_data,
    static_cast<int>(shape[1]),
    static_cast<int>(shape[2]),
    seg_class_names_,
    preprocess_info_);
}

}  // namespace bitbots_vision
