#include <bitbots_vision/rfdetr_processing.hpp>
#include <bitbots_vision/tensorrt_handler.hpp>
#include <filesystem>
#include <fstream>
#include <rclcpp/logging.hpp>
#include <stdexcept>

namespace bitbots_vision {

namespace {

void check_cuda(cudaError_t err, const char* what) {
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string(what) + " failed: " + cudaGetErrorString(err));
  }
}

int64_t dims_element_count(const nvinfer1::Dims& dims) {
  int64_t count = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    count *= dims.d[i];
  }
  return count;
}

}  // namespace

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

TensorRtHandler::TensorRtHandler(const std::string& model_path, const ModelConfig& model_config,
                                 const RfdetrConfig& cfg, const rclcpp::Logger& logger)
    : trt_logger_(logger), cfg_(cfg), logger_(logger) {
  det_class_names_ = model_config.detection_classes();
  build_or_load_engine(model_path);
  allocate_buffers();
}

TensorRtHandler::~TensorRtHandler() {
  if (d_input_) {
    cudaFree(d_input_);
  }
  if (d_dets_) {
    cudaFree(d_dets_);
  }
  if (d_labels_) {
    cudaFree(d_labels_);
  }
  if (d_line_mask_) {
    cudaFree(d_line_mask_);
  }
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

// ---------------------------------------------------------------------------
// Engine build / load
// ---------------------------------------------------------------------------

void TensorRtHandler::build_or_load_engine(const std::string& model_path) {
  const std::string onnx_path = model_path + "/onnx/rfdetr.onnx";
  const std::string engine_cache_path = model_path + "/onnx/rfdetr.trt";

  if (!std::filesystem::exists(onnx_path)) {
    throw std::runtime_error("ONNX model file not found: " + onnx_path);
  }

  const bool cache_is_fresh = std::filesystem::exists(engine_cache_path) &&
                              std::filesystem::last_write_time(engine_cache_path) >=
                                  std::filesystem::last_write_time(onnx_path);

  if (cache_is_fresh) {
    RCLCPP_INFO(logger_, "Loading cached TensorRT engine: %s", engine_cache_path.c_str());
    std::ifstream file(engine_cache_path, std::ios::binary | std::ios::ate);
    const std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> engine_data(static_cast<size_t>(size > 0 ? size : 0));
    if (size <= 0 || !file.read(engine_data.data(), size)) {
      throw std::runtime_error("Failed to read cached TensorRT engine: " + engine_cache_path);
    }

    runtime_.reset(nvinfer1::createInferRuntime(trt_logger_));
    if (!runtime_) {
      throw std::runtime_error("Failed to create TensorRT runtime");
    }
    engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size()));
    if (!engine_) {
      throw std::runtime_error(
          "Failed to deserialize cached TensorRT engine '" + engine_cache_path +
          "' (likely built for a different TensorRT version or GPU) -- delete it to force a rebuild");
    }
  } else {
    RCLCPP_INFO(logger_, "Building TensorRT engine from %s (this can take several minutes on first run)...",
                onnx_path.c_str());

    std::unique_ptr<nvinfer1::IBuilder, TrtDeleter> builder(nvinfer1::createInferBuilder(trt_logger_));
    if (!builder) {
      throw std::runtime_error("Failed to create TensorRT builder");
    }

    const auto explicit_batch =
        1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    std::unique_ptr<nvinfer1::INetworkDefinition, TrtDeleter> network(builder->createNetworkV2(explicit_batch));
    if (!network) {
      throw std::runtime_error("Failed to create TensorRT network definition");
    }

    std::unique_ptr<nvonnxparser::IParser, TrtDeleter> parser(nvonnxparser::createParser(*network, trt_logger_));
    if (!parser) {
      throw std::runtime_error("Failed to create ONNX parser");
    }
    if (!parser->parseFromFile(onnx_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
      for (int32_t i = 0; i < parser->getNbErrors(); ++i) {
        RCLCPP_ERROR(logger_, "ONNX parse error: %s", parser->getError(i)->desc());
      }
      throw std::runtime_error("Failed to parse ONNX model: " + onnx_path);
    }

    std::unique_ptr<nvinfer1::IBuilderConfig, TrtDeleter> config(builder->createBuilderConfig());
    if (!config) {
      throw std::runtime_error("Failed to create TensorRT builder config");
    }
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 30);  // 1 GiB

    if (builder->platformHasFastFp16()) {
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
      RCLCPP_INFO(logger_, "TensorRT: platform supports fast FP16, enabling FP16 mode");
    } else {
      RCLCPP_WARN(logger_, "TensorRT: platform has no fast FP16 support, building FP32-only engine");
    }

    std::unique_ptr<nvinfer1::IHostMemory, TrtDeleter> serialized(builder->buildSerializedNetwork(*network, *config));
    if (!serialized) {
      throw std::runtime_error("TensorRT engine build failed for " + onnx_path);
    }

    std::ofstream out(engine_cache_path, std::ios::binary);
    out.write(reinterpret_cast<const char*>(serialized->data()), static_cast<std::streamsize>(serialized->size()));
    out.close();
    RCLCPP_INFO(logger_, "Cached TensorRT engine to %s", engine_cache_path.c_str());

    runtime_.reset(nvinfer1::createInferRuntime(trt_logger_));
    if (!runtime_) {
      throw std::runtime_error("Failed to create TensorRT runtime");
    }
    engine_.reset(runtime_->deserializeCudaEngine(serialized->data(), serialized->size()));
    if (!engine_) {
      throw std::runtime_error("Failed to deserialize freshly-built TensorRT engine");
    }
  }

  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("Failed to create TensorRT execution context");
  }

  check_cuda(cudaStreamCreate(&stream_), "cudaStreamCreate");

  input_binding_idx_ = engine_->getBindingIndex("input");
  dets_binding_idx_ = engine_->getBindingIndex("dets");
  labels_binding_idx_ = engine_->getBindingIndex("labels");
  line_mask_binding_idx_ = engine_->getBindingIndex("line_mask");

  if (input_binding_idx_ < 0 || dets_binding_idx_ < 0 || labels_binding_idx_ < 0) {
    throw std::runtime_error("TensorRT engine is missing a required 'input', 'dets', or 'labels' binding");
  }
  if (line_mask_binding_idx_ < 0) {
    RCLCPP_WARN(logger_, "TensorRT engine has no 'line_mask' binding -- line segmentation will be empty");
  }

  const nvinfer1::Dims input_dims = engine_->getBindingDimensions(input_binding_idx_);
  if (input_dims.nbDims == 4) {
    net_h_ = input_dims.d[2];
    net_w_ = input_dims.d[3];
  }
  if (net_h_ != 384 || net_w_ != 384) {
    RCLCPP_WARN(logger_, "TensorRT engine input is %dx%d, expected 384x384 -- proceeding anyway", net_h_, net_w_);
  }

  RCLCPP_INFO(logger_, "TensorRT engine ready -- input [1,3,%d,%d], line_mask: %s", net_h_, net_w_,
              line_mask_binding_idx_ >= 0 ? "yes" : "no");
}

// ---------------------------------------------------------------------------
// Buffer allocation
// ---------------------------------------------------------------------------

void TensorRtHandler::allocate_buffers() {
  const nvinfer1::Dims dets_dims = engine_->getBindingDimensions(dets_binding_idx_);
  const nvinfer1::Dims labels_dims = engine_->getBindingDimensions(labels_binding_idx_);

  num_dets_ = dets_dims.d[1];
  num_classes_ = labels_dims.d[2];

  check_cuda(cudaMalloc(&d_input_, sizeof(float) * 3 * static_cast<size_t>(net_h_) * static_cast<size_t>(net_w_)),
             "cudaMalloc(input)");
  check_cuda(cudaMalloc(&d_dets_, sizeof(float) * static_cast<size_t>(dims_element_count(dets_dims))),
             "cudaMalloc(dets)");
  check_cuda(cudaMalloc(&d_labels_, sizeof(float) * static_cast<size_t>(dims_element_count(labels_dims))),
             "cudaMalloc(labels)");

  if (line_mask_binding_idx_ >= 0) {
    const nvinfer1::Dims lm_dims = engine_->getBindingDimensions(line_mask_binding_idx_);
    if (lm_dims.nbDims == 4) {
      line_mask_h_ = lm_dims.d[2];
      line_mask_w_ = lm_dims.d[3];
    }
    check_cuda(cudaMalloc(&d_line_mask_, sizeof(float) * static_cast<size_t>(dims_element_count(lm_dims))),
               "cudaMalloc(line_mask)");
  }
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void TensorRtHandler::reconfigure(const RfdetrConfig& cfg) { cfg_ = cfg; }

void TensorRtHandler::set_image(const cv::Mat& bgr_image) {
  det_results_.clear();
  line_mask_result_ = cv::Mat();
  preprocess(bgr_image);
  prediction_is_fresh_ = false;
}

void TensorRtHandler::predict() {
  if (prediction_is_fresh_) {
    return;
  }
  run_inference();
  prediction_is_fresh_ = true;
}

std::vector<Candidate> TensorRtHandler::get_detection_candidates_for(const std::string& class_name) {
  predict();
  auto it = det_results_.find(class_name);
  return (it != det_results_.end()) ? it->second : std::vector<Candidate>{};
}

cv::Mat TensorRtHandler::get_line_mask() {
  predict();
  return line_mask_result_;
}

const std::vector<std::string>& TensorRtHandler::detection_class_names() const { return det_class_names_; }

// ---------------------------------------------------------------------------
// Preprocessing (shared, backend-agnostic implementation)
// ---------------------------------------------------------------------------

void TensorRtHandler::preprocess(const cv::Mat& bgr_image) {
  orig_h_ = bgr_image.rows;
  orig_w_ = bgr_image.cols;
  input_data_ = processing::preprocess_image_rfdetr(bgr_image, net_h_, net_w_);
}

// ---------------------------------------------------------------------------
// Inference
// ---------------------------------------------------------------------------

void TensorRtHandler::run_inference() {
  check_cuda(cudaMemcpyAsync(d_input_, input_data_.data(), input_data_.size() * sizeof(float),
                             cudaMemcpyHostToDevice, stream_),
             "cudaMemcpyAsync(input H2D)");

  std::vector<void*> bindings(static_cast<size_t>(engine_->getNbBindings()), nullptr);
  bindings[static_cast<size_t>(input_binding_idx_)] = d_input_;
  bindings[static_cast<size_t>(dets_binding_idx_)] = d_dets_;
  bindings[static_cast<size_t>(labels_binding_idx_)] = d_labels_;
  if (line_mask_binding_idx_ >= 0) {
    bindings[static_cast<size_t>(line_mask_binding_idx_)] = d_line_mask_;
  }

  if (!context_->enqueueV2(bindings.data(), stream_, nullptr)) {
    throw std::runtime_error("TensorRT inference (enqueueV2) failed");
  }

  std::vector<float> dets_host(static_cast<size_t>(num_dets_ * 4));
  std::vector<float> labels_host(static_cast<size_t>(num_dets_ * num_classes_));
  check_cuda(cudaMemcpyAsync(dets_host.data(), d_dets_, dets_host.size() * sizeof(float), cudaMemcpyDeviceToHost,
                             stream_),
             "cudaMemcpyAsync(dets D2H)");
  check_cuda(cudaMemcpyAsync(labels_host.data(), d_labels_, labels_host.size() * sizeof(float),
                             cudaMemcpyDeviceToHost, stream_),
             "cudaMemcpyAsync(labels D2H)");

  std::vector<float> line_mask_host;
  if (line_mask_binding_idx_ >= 0) {
    line_mask_host.resize(static_cast<size_t>(line_mask_h_) * static_cast<size_t>(line_mask_w_));
    check_cuda(cudaMemcpyAsync(line_mask_host.data(), d_line_mask_, line_mask_host.size() * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_),
               "cudaMemcpyAsync(line_mask D2H)");
  }

  check_cuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");

  // Same pure, backend-agnostic postprocessing used by the ONNX Runtime
  // handler -- already unit-tested and numerically verified against the
  // Python reference implementation.
  det_results_ = processing::postprocess_detections_rfdetr(dets_host.data(), labels_host.data(), num_dets_,
                                                            num_classes_, det_class_names_,
                                                            cfg_.class_conf_thresholds, cfg_.default_conf_threshold,
                                                            orig_h_, orig_w_);

  if (line_mask_binding_idx_ >= 0) {
    line_mask_result_ = processing::postprocess_line_mask_rfdetr(line_mask_host.data(), line_mask_h_, line_mask_w_,
                                                                  cfg_.line_mask_threshold, orig_h_, orig_w_);
  }
}

}  // namespace bitbots_vision
