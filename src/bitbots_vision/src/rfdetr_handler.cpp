#include <algorithm>
#include <bitbots_vision/rfdetr_handler.hpp>
#include <bitbots_vision/rfdetr_processing.hpp>
#include <filesystem>
#include <rclcpp/logging.hpp>
#include <stdexcept>

namespace bitbots_vision {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

RfdetrHandler::RfdetrHandler(const std::string& model_path, const ModelConfig& model_config, const Config& cfg,
                             const rclcpp::Logger& logger)
    : env_(ORT_LOGGING_LEVEL_WARNING, "bitbots_vision"), cfg_(cfg), logger_(logger) {
  det_class_names_ = model_config.detection_classes();

  init_session(model_path);
}

// ---------------------------------------------------------------------------
// init_session – builds the ONNX session with provider fallback chain
// ---------------------------------------------------------------------------

void RfdetrHandler::init_session(const std::string& model_path) {
  const std::string onnx_file = model_path + "/onnx/rfdetr.onnx";
  if (!std::filesystem::exists(onnx_file)) {
    throw std::runtime_error("ONNX model file not found: " + onnx_file);
  }

  session_options_.SetIntraOpNumThreads(1);
  session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  // Register execution providers in priority order.
  // Ort::GetAvailableProviders() returns providers compiled into this ORT build.
  // CUDA and TensorRT require typed registration APIs; other providers use the generic API.
  const auto available = Ort::GetAvailableProviders();
  auto has_ep = [&](const std::string& name) {
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
    } catch (const Ort::Exception& e) {
      RCLCPP_WARN(logger_, "TensorrtExecutionProvider unavailable: %s", e.what());
    }
  }

  if (has_ep("CUDAExecutionProvider")) {
    try {
      OrtCUDAProviderOptions cuda_opts{};
      session_options_.AppendExecutionProvider_CUDA(cuda_opts);
      RCLCPP_INFO(logger_, "ONNX Runtime: registered CUDAExecutionProvider");
    } catch (const Ort::Exception& e) {
      RCLCPP_WARN(logger_, "CUDAExecutionProvider unavailable: %s", e.what());
    }
  }

  if (has_ep("WebGpuExecutionProvider")) {
    try {
      session_options_.AppendExecutionProvider("WebGPU", {});
      RCLCPP_INFO(logger_, "ONNX Runtime: registered WebGpuExecutionProvider");
    } catch (const Ort::Exception& e) {
      RCLCPP_WARN(logger_, "WebGpuExecutionProvider unavailable: %s", e.what());
    }
  }

  // List available providers for debugging
  RCLCPP_INFO(logger_, "Available ONNX Runtime execution providers:");
  for (const auto& ep : available) {
    RCLCPP_INFO(logger_, "  %s", ep.c_str());
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
  // Expected shape: [1, 3, H_net, W_net], fixed at 384x384 for the current RF-DETR export.
  if (input_shape_.size() == 4) {
    if (input_shape_[0] < 0) {
      input_shape_[0] = 1;
    }
    if (input_shape_[2] != 384 || input_shape_[3] != 384) {
      RCLCPP_WARN(logger_, "RF-DETR model input is %lldx%lld, expected 384x384 — proceeding anyway",
                  static_cast<long long>(input_shape_[2]), static_cast<long long>(input_shape_[3]));
    }
  }

  // Outputs — accessed positionally later (dets, then logits), matching the reference
  // Python implementation's session.run(None, ...) behaviour.
  const size_t num_outputs = session_->GetOutputCount();
  for (size_t i = 0; i < num_outputs; ++i) {
    auto out_name_ptr = session_->GetOutputNameAllocated(i, allocator);
    output_names_.emplace_back(out_name_ptr.get());
  }

  RCLCPP_INFO(logger_, "Model loaded – input '%s' [1, 3, %lld, %lld], %zu outputs", input_name_.c_str(),
              static_cast<long long>(input_shape_[2]), static_cast<long long>(input_shape_[3]), num_outputs);
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void RfdetrHandler::reconfigure(const Config& cfg) { cfg_ = cfg; }

void RfdetrHandler::set_image(const cv::Mat& bgr_image) {
  det_results_.clear();
  preprocess(bgr_image);
  prediction_is_fresh_ = false;
}

void RfdetrHandler::predict() {
  if (prediction_is_fresh_) {
    return;
  }
  run_inference();
  prediction_is_fresh_ = true;
}

std::vector<Candidate> RfdetrHandler::get_detection_candidates_for(const std::string& class_name) {
  predict();
  auto it = det_results_.find(class_name);
  return (it != det_results_.end()) ? it->second : std::vector<Candidate>{};
}

const std::vector<std::string>& RfdetrHandler::detection_class_names() const { return det_class_names_; }

// ---------------------------------------------------------------------------
// Preprocessing  (BGR uint8 → RGB float32, direct resize, CHW)
// ---------------------------------------------------------------------------

void RfdetrHandler::preprocess(const cv::Mat& bgr_image) {
  orig_h_ = bgr_image.rows;
  orig_w_ = bgr_image.cols;

  const int net_h = static_cast<int>(input_shape_[2]);
  const int net_w = static_cast<int>(input_shape_[3]);
  input_data_ = processing::preprocess_image_rfdetr(bgr_image, net_h, net_w);
  input_shape_[0] = 1;  // batch dimension is always 1
}

// ---------------------------------------------------------------------------
// Inference
// ---------------------------------------------------------------------------

void RfdetrHandler::run_inference() {
  auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_data_.data(), input_data_.size(),
                                                            input_shape_.data(), input_shape_.size());

  const char* input_name_cstr = input_name_.c_str();
  std::vector<const char*> out_name_cstrs;
  for (const auto& n : output_names_) {
    out_name_cstrs.push_back(n.c_str());
  }

  auto outputs = session_->Run(Ort::RunOptions{nullptr}, &input_name_cstr, &input_tensor, 1, out_name_cstrs.data(),
                               out_name_cstrs.size());

  if (outputs.size() < 2) {
    RCLCPP_ERROR(logger_, "Expected at least 2 outputs from RF-DETR model, got %zu", outputs.size());
    return;
  }

  // Positional: outputs[0] = dets [1, N, 4], outputs[1] = logits [1, N, num_classes]
  const auto dets_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
  const auto logits_shape = outputs[1].GetTensorTypeAndShapeInfo().GetShape();
  if (dets_shape.size() < 3 || logits_shape.size() < 3) {
    RCLCPP_ERROR(logger_, "Unexpected RF-DETR output tensor rank");
    return;
  }

  const int64_t num_dets = dets_shape[1];
  const int64_t num_classes = logits_shape[2];

  det_results_ = processing::postprocess_detections_rfdetr(
      outputs[0].GetTensorData<float>(), outputs[1].GetTensorData<float>(), num_dets, num_classes, det_class_names_,
      cfg_.class_conf_thresholds, cfg_.default_conf_threshold, orig_h_, orig_w_);
}

}  // namespace bitbots_vision
