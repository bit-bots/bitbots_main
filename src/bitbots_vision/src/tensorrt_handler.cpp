#include <bitbots_vision/onnx_layernorm_decomposition.hpp>
#include <bitbots_vision/rfdetr_processing.hpp>
#include <bitbots_vision/tensorrt_handler.hpp>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <rclcpp/logging.hpp>
#include <stdexcept>

namespace bitbots_vision {

namespace {

void check_cuda(cudaError_t err, const char* what) {
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string(what) + " failed: " + cudaGetErrorString(err));
  }
}

// Increment whenever the engine build settings change (precision choices,
// graph rewrites, ...) so cached engines built with the old settings are
// rebuilt instead of reused.
constexpr int kEngineBuildRevision = 2;

// LayerNormalizations under this module (RF-DETR's line-segmentation head)
// stay in FP16 entirely. Measured on recorded game frames, their FP32
// variance computation adds about a millisecond per frame (it forces format
// conversions on the head's large feature maps) without any measurable
// change in the line mask or the detections.
constexpr const char* kFp16LayerNormModule = "/line_segmentation_head/";

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
  if (graph_exec_) {
    cudaGraphExecDestroy(graph_exec_);
  }
  for (float* host_buffer : {h_input_, h_dets_, h_labels_, h_line_mask_}) {
    if (host_buffer) {
      cudaFreeHost(host_buffer);
    }
  }
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
  // The cache name encodes the TensorRT version and the engine build
  // revision, so an engine built by another TensorRT version or with
  // outdated build settings is never loaded.
  const std::string engine_cache_path = model_path + "/onnx/rfdetr_trt" + std::to_string(NV_TENSORRT_MAJOR) + "." +
                                        std::to_string(NV_TENSORRT_MINOR) + "." + std::to_string(NV_TENSORRT_PATCH) +
                                        "_r" + std::to_string(kEngineBuildRevision) + ".engine";

  if (!std::filesystem::exists(onnx_path)) {
    throw std::runtime_error("ONNX model file not found: " + onnx_path);
  }

  const bool cache_is_fresh =
      std::filesystem::exists(engine_cache_path) &&
      std::filesystem::last_write_time(engine_cache_path) >= std::filesystem::last_write_time(onnx_path);

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

    const auto explicit_batch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    std::unique_ptr<nvinfer1::INetworkDefinition, TrtDeleter> network(builder->createNetworkV2(explicit_batch));
    if (!network) {
      throw std::runtime_error("Failed to create TensorRT network definition");
    }

    std::unique_ptr<nvonnxparser::IParser, TrtDeleter> parser(nvonnxparser::createParser(*network, trt_logger_));
    if (!parser) {
      throw std::runtime_error("Failed to create ONNX parser");
    }
    // TensorRT < 8.6 has no importer for the opset-17 LayerNormalization op,
    // so it is decomposed into primitive ops before parsing.
    std::ifstream onnx_file(onnx_path, std::ios::binary);
    const std::string onnx_bytes((std::istreambuf_iterator<char>(onnx_file)), std::istreambuf_iterator<char>());
    if (!onnx_file.good() && !onnx_file.eof()) {
      throw std::runtime_error("Failed to read ONNX model file: " + onnx_path);
    }
    const auto decomposition = onnx_rewrite::decompose_layer_normalization(onnx_bytes);
    if (decomposition.num_decomposed > 0 || decomposition.num_skipped > 0) {
      RCLCPP_INFO(logger_, "Decomposed %d LayerNormalization node(s) into primitive ops (%d left unchanged)",
                  decomposition.num_decomposed, decomposition.num_skipped);
    }

    if (!parser->parse(decomposition.model.data(), decomposition.model.size(), onnx_path.c_str())) {
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
      // The variance computation of decomposed LayerNormalizations overflows
      // or loses too much precision in FP16 (detections break), so keep
      // those layers in FP32. Pinning more of the decomposition costs speed
      // without measurably improving accuracy.
      int fp32_layers = 0;
      for (int32_t i = 0; i < network->getNbLayers(); ++i) {
        nvinfer1::ILayer* layer = network->getLayer(i);
        // Constants keep their type, and ONNX Cast nodes are imported as
        // identity layers whose output type *is* the cast, so both are skipped.
        if (!onnx_rewrite::is_precision_sensitive_layer(layer->getName()) ||
            std::strstr(layer->getName(), kFp16LayerNormModule) != nullptr ||
            layer->getType() == nvinfer1::LayerType::kCONSTANT || layer->getType() == nvinfer1::LayerType::kIDENTITY) {
          continue;
        }
        layer->setPrecision(nvinfer1::DataType::kFLOAT);
        for (int32_t o = 0; o < layer->getNbOutputs(); ++o) {
          layer->setOutputType(o, nvinfer1::DataType::kFLOAT);
        }
        ++fp32_layers;
      }
      if (fp32_layers > 0) {
        config->setFlag(nvinfer1::BuilderFlag::kPREFER_PRECISION_CONSTRAINTS);
        RCLCPP_INFO(logger_, "TensorRT: keeping %d LayerNormalization variance layer(s) in FP32", fp32_layers);
      }
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

  input_size_ = 3 * static_cast<size_t>(net_h_) * static_cast<size_t>(net_w_);
  dets_size_ = static_cast<size_t>(dims_element_count(dets_dims));
  labels_size_ = static_cast<size_t>(dims_element_count(labels_dims));

  check_cuda(cudaMalloc(&d_input_, sizeof(float) * input_size_), "cudaMalloc(input)");
  check_cuda(cudaMalloc(&d_dets_, sizeof(float) * dets_size_), "cudaMalloc(dets)");
  check_cuda(cudaMalloc(&d_labels_, sizeof(float) * labels_size_), "cudaMalloc(labels)");
  check_cuda(cudaMallocHost(reinterpret_cast<void**>(&h_input_), sizeof(float) * input_size_), "cudaMallocHost(input)");
  check_cuda(cudaMallocHost(reinterpret_cast<void**>(&h_dets_), sizeof(float) * dets_size_), "cudaMallocHost(dets)");
  check_cuda(cudaMallocHost(reinterpret_cast<void**>(&h_labels_), sizeof(float) * labels_size_),
             "cudaMallocHost(labels)");

  if (line_mask_binding_idx_ >= 0) {
    const nvinfer1::Dims lm_dims = engine_->getBindingDimensions(line_mask_binding_idx_);
    if (lm_dims.nbDims == 4) {
      line_mask_h_ = lm_dims.d[2];
      line_mask_w_ = lm_dims.d[3];
    }
    line_mask_size_ = static_cast<size_t>(dims_element_count(lm_dims));
    check_cuda(cudaMalloc(&d_line_mask_, sizeof(float) * line_mask_size_), "cudaMalloc(line_mask)");
    check_cuda(cudaMallocHost(reinterpret_cast<void**>(&h_line_mask_), sizeof(float) * line_mask_size_),
               "cudaMallocHost(line_mask)");
  }

  bindings_.assign(static_cast<size_t>(engine_->getNbBindings()), nullptr);
  bindings_[static_cast<size_t>(input_binding_idx_)] = d_input_;
  bindings_[static_cast<size_t>(dets_binding_idx_)] = d_dets_;
  bindings_[static_cast<size_t>(labels_binding_idx_)] = d_labels_;
  if (line_mask_binding_idx_ >= 0) {
    bindings_[static_cast<size_t>(line_mask_binding_idx_)] = d_line_mask_;
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
  const std::vector<float> input = processing::preprocess_image_rfdetr(bgr_image, net_h_, net_w_);
  std::memcpy(h_input_, input.data(), input_size_ * sizeof(float));
}

// ---------------------------------------------------------------------------
// Inference
// ---------------------------------------------------------------------------

void TensorRtHandler::enqueue_inference() {
  check_cuda(cudaMemcpyAsync(d_input_, h_input_, input_size_ * sizeof(float), cudaMemcpyHostToDevice, stream_),
             "cudaMemcpyAsync(input H2D)");
  if (!context_->enqueueV2(bindings_.data(), stream_, nullptr)) {
    throw std::runtime_error("TensorRT inference (enqueueV2) failed");
  }
  check_cuda(cudaMemcpyAsync(h_dets_, d_dets_, dets_size_ * sizeof(float), cudaMemcpyDeviceToHost, stream_),
             "cudaMemcpyAsync(dets D2H)");
  check_cuda(cudaMemcpyAsync(h_labels_, d_labels_, labels_size_ * sizeof(float), cudaMemcpyDeviceToHost, stream_),
             "cudaMemcpyAsync(labels D2H)");
  if (line_mask_binding_idx_ >= 0) {
    check_cuda(
        cudaMemcpyAsync(h_line_mask_, d_line_mask_, line_mask_size_ * sizeof(float), cudaMemcpyDeviceToHost, stream_),
        "cudaMemcpyAsync(line_mask D2H)");
  }
}

void TensorRtHandler::capture_graph() {
  graph_capture_attempted_ = true;
  cudaGraph_t graph = nullptr;
  if (cudaStreamBeginCapture(stream_, cudaStreamCaptureModeThreadLocal) != cudaSuccess) {
    RCLCPP_WARN(logger_, "TensorRT: CUDA graph capture unavailable, enqueueing inference directly");
    cudaGetLastError();  // clear the error state
    return;
  }
  bool enqueued = true;
  try {
    enqueue_inference();
  } catch (const std::runtime_error&) {
    enqueued = false;
  }
  const cudaError_t end_status = cudaStreamEndCapture(stream_, &graph);
  if (enqueued && end_status == cudaSuccess &&
      cudaGraphInstantiate(&graph_exec_, graph, nullptr, nullptr, 0) == cudaSuccess) {
    RCLCPP_INFO(logger_, "TensorRT: inference captured into a CUDA graph");
  } else {
    graph_exec_ = nullptr;
    cudaGetLastError();  // clear the error state
    RCLCPP_WARN(logger_, "TensorRT: CUDA graph capture failed, enqueueing inference directly");
  }
  if (graph) {
    cudaGraphDestroy(graph);
  }
}

void TensorRtHandler::run_inference() {
  if (graph_exec_) {
    check_cuda(cudaGraphLaunch(graph_exec_, stream_), "cudaGraphLaunch");
  } else {
    enqueue_inference();
  }
  check_cuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");

  // TensorRT needs one regular enqueue before the context's work can be
  // captured into a graph, so capturing happens after the first inference.
  if (!graph_capture_attempted_) {
    capture_graph();
  }

  // Same pure, backend-agnostic postprocessing used by the ONNX Runtime
  // handler -- already unit-tested and numerically verified against the
  // Python reference implementation.
  det_results_ = processing::postprocess_detections_rfdetr(h_dets_, h_labels_, num_dets_, num_classes_,
                                                           det_class_names_, cfg_.class_conf_thresholds,
                                                           cfg_.default_conf_threshold, orig_h_, orig_w_);

  if (line_mask_binding_idx_ >= 0) {
    line_mask_result_ = processing::postprocess_line_mask_rfdetr(h_line_mask_, line_mask_h_, line_mask_w_,
                                                                 cfg_.line_mask_threshold, orig_h_, orig_w_);
  }
}

}  // namespace bitbots_vision
