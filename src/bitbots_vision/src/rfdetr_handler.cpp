#include <algorithm>
#include <bitbots_vision/rfdetr_handler.hpp>
#include <bitbots_vision/rfdetr_processing.hpp>
#include <cstdlib>
#include <filesystem>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>

namespace bitbots_vision {

namespace {
// Allows forcing CPU-only inference at runtime (no rebuild needed), e.g. for
// dev-machine debugging or to work around a flaky/unavailable GPU EP on the robot.
bool cpu_fallback_forced() {
  const char* val = std::getenv("BITBOTS_VISION_FORCE_CPU");
  return val != nullptr && std::string(val) != "0" && std::string(val) != "";
}
}  // namespace

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

Ort::SessionOptions RfdetrHandler::build_session_options(bool force_cpu) const {
  Ort::SessionOptions session_options;
  session_options.SetIntraOpNumThreads(1);
  session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  // Register execution providers in priority order.
  // Ort::GetAvailableProviders() returns providers compiled into this ORT build.
  // CUDA and TensorRT require typed registration APIs; other providers use the generic API.
  const auto available = Ort::GetAvailableProviders();
  auto has_ep = [&](const std::string& name) {
    return std::find(available.begin(), available.end(), name) != available.end();
  };

  if (force_cpu) {
    RCLCPP_WARN(logger_, "Skipping all GPU execution providers, using CPU only");
  } else {
    // TRT and CUDA use typed registration APIs and dynamically load their provider shared
    // libraries at registration time. Wrap in try/catch so a missing system library
    // (e.g. libcublasLt.so) causes graceful fallback rather than a crash.
    if (has_ep("TensorrtExecutionProvider")) {
      try {
        OrtTensorRTProviderOptions trt_opts{};
        session_options.AppendExecutionProvider_TensorRT(trt_opts);
        RCLCPP_INFO(logger_, "ONNX Runtime: registered TensorrtExecutionProvider");
      } catch (const Ort::Exception& e) {
        RCLCPP_WARN(logger_, "TensorrtExecutionProvider unavailable: %s", e.what());
      }
    }

    if (has_ep("CUDAExecutionProvider")) {
      try {
        OrtCUDAProviderOptions cuda_opts{};
        session_options.AppendExecutionProvider_CUDA(cuda_opts);
        RCLCPP_INFO(logger_, "ONNX Runtime: registered CUDAExecutionProvider");
      } catch (const Ort::Exception& e) {
        RCLCPP_WARN(logger_, "CUDAExecutionProvider unavailable: %s", e.what());
      }
    }

    if (has_ep("WebGpuExecutionProvider")) {
      try {
        session_options.AppendExecutionProvider("WebGPU", {});
        RCLCPP_INFO(logger_, "ONNX Runtime: registered WebGpuExecutionProvider");
      } catch (const Ort::Exception& e) {
        RCLCPP_WARN(logger_, "WebGpuExecutionProvider unavailable: %s", e.what());
      }
    }
  }

  // List available providers for debugging
  RCLCPP_INFO(logger_, "Available ONNX Runtime execution providers:");
  for (const auto& ep : available) {
    RCLCPP_INFO(logger_, "  %s", ep.c_str());
  }

  // CPU is always the implicit fallback — no explicit registration needed.
  return session_options;
}

void RfdetrHandler::init_session(const std::string& model_path) {
  const std::string onnx_file = model_path + "/onnx/rfdetr.onnx";
  if (!std::filesystem::exists(onnx_file)) {
    throw std::runtime_error("ONNX model file not found: " + onnx_file);
  }

  const bool force_cpu = cpu_fallback_forced();
  if (force_cpu) {
    RCLCPP_WARN(logger_, "BITBOTS_VISION_FORCE_CPU is set — forcing CPU-only inference");
  }

  RCLCPP_INFO(logger_, "Loading ONNX model: %s", onnx_file.c_str());
  try {
    Ort::SessionOptions session_options = build_session_options(force_cpu);
    session_ = std::make_unique<Ort::Session>(env_, onnx_file.c_str(), session_options);
  } catch (const Ort::Exception& e) {
    if (force_cpu) {
      throw;  // Already CPU-only — nothing left to fall back to.
    }
    // A registered GPU EP can fail not just at registration time but also
    // later, when ORT actually compiles the graph for it during session
    // creation (e.g. a WebGPU shader requiring f16 on a device without f16
    // support). That failure throws out of the Session constructor itself,
    // so it has to be caught here rather than around AppendExecutionProvider.
    RCLCPP_ERROR(logger_, "Session creation with GPU execution providers failed (%s) — retrying CPU-only", e.what());
    Ort::SessionOptions cpu_only_options = build_session_options(true);
    session_ = std::make_unique<Ort::Session>(env_, onnx_file.c_str(), cpu_only_options);
  }

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

  // Outputs — resolved by name (not position), since the output set/order
  // can vary between model exports (e.g. an older export without a
  // "line_mask" output).
  const size_t num_outputs = session_->GetOutputCount();
  for (size_t i = 0; i < num_outputs; ++i) {
    auto out_name_ptr = session_->GetOutputNameAllocated(i, allocator);
    output_names_.emplace_back(out_name_ptr.get());
    if (output_names_.back() == "dets") {
      dets_idx_ = static_cast<int>(i);
    } else if (output_names_.back() == "labels") {
      labels_idx_ = static_cast<int>(i);
    } else if (output_names_.back() == "line_mask") {
      line_mask_idx_ = static_cast<int>(i);
    }
  }

  if (dets_idx_ < 0 || labels_idx_ < 0) {
    throw std::runtime_error("RF-DETR model is missing a required 'dets' or 'labels' output");
  }
  if (line_mask_idx_ < 0) {
    RCLCPP_WARN(logger_, "RF-DETR model has no 'line_mask' output — line segmentation will be empty");
  }

  RCLCPP_INFO(logger_, "Model loaded – input '%s' [1, 3, %lld, %lld], %zu outputs (line_mask: %s)", input_name_.c_str(),
              static_cast<long long>(input_shape_[2]), static_cast<long long>(input_shape_[3]), num_outputs,
              line_mask_idx_ >= 0 ? "yes" : "no");
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void RfdetrHandler::reconfigure(const Config& cfg) { cfg_ = cfg; }

void RfdetrHandler::set_image(const cv::Mat& bgr_image) {
  det_results_.clear();
  line_mask_result_ = cv::Mat();
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

cv::Mat RfdetrHandler::get_line_mask() {
  predict();
  return line_mask_result_;
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

  const auto& dets_out = outputs[static_cast<size_t>(dets_idx_)];
  const auto& labels_out = outputs[static_cast<size_t>(labels_idx_)];

  const auto dets_shape = dets_out.GetTensorTypeAndShapeInfo().GetShape();
  const auto labels_shape = labels_out.GetTensorTypeAndShapeInfo().GetShape();
  if (dets_shape.size() < 3 || labels_shape.size() < 3) {
    RCLCPP_ERROR(logger_, "Unexpected RF-DETR output tensor rank");
    return;
  }

  const int64_t num_dets = dets_shape[1];
  const int64_t num_classes = labels_shape[2];

  det_results_ = processing::postprocess_detections_rfdetr(
      dets_out.GetTensorData<float>(), labels_out.GetTensorData<float>(), num_dets, num_classes, det_class_names_,
      cfg_.class_conf_thresholds, cfg_.default_conf_threshold, orig_h_, orig_w_);

  if (line_mask_idx_ >= 0) {
    const auto& line_mask_out = outputs[static_cast<size_t>(line_mask_idx_)];
    const auto line_mask_shape = line_mask_out.GetTensorTypeAndShapeInfo().GetShape();
    if (line_mask_shape.size() == 4) {
      const int mask_h = static_cast<int>(line_mask_shape[2]);
      const int mask_w = static_cast<int>(line_mask_shape[3]);
      line_mask_result_ = processing::postprocess_line_mask_rfdetr(line_mask_out.GetTensorData<float>(), mask_h, mask_w,
                                                                   cfg_.line_mask_threshold, orig_h_, orig_w_);
    } else {
      RCLCPP_ERROR(logger_, "Unexpected 'line_mask' output tensor rank");
    }
  }
}

}  // namespace bitbots_vision
