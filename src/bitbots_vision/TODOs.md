# TODOs: RF-DETR TensorRT performance

Goal: run the vision at the full camera rate of 60 FPS on the robots (Jetson Orin NX,
JetPack 5, TensorRT 8.5) with the native TensorRT backend (`TensorRtHandler`).

## Status

Measured on Peter (2026-11-07) with the ZED publishing at 60 FPS:

| State | GPU time per inference (`trtexec`) | Vision output rate |
|---|---|---|
| Initial TensorRT backend | 23.3 ms | 32 FPS |
| CUDA graph + only the LayerNorm variance path in FP32 (current) | 18.8 ms | 39 FPS |

- Accuracy of the current engine was checked against ONNX Runtime on 8 frames from a game recording:
  all 13 reference detections found, max box deviation 0.3 px, line mask IoU ≥ 0.99 on frames with lines.
- An all-FP16 engine is only marginally faster but breaks the detections (1/13 found),
  so the FP32 variance path in the LayerNormalizations is required.
- The GPU is already at its maximum clock in MAXN mode, so clock tuning has nothing left to give.
- The network alone limits the rate to about 53 FPS; the remaining gap to the measured rate is
  per-frame CPU work in the node, which runs serially with the inference.

## Next steps

### 1. Profile and reduce the node overhead

The node takes several milliseconds per frame more than the inference alone.
Profile the stages of `VisionNode::image_callback` on the robot (e.g. temporary `std::chrono` scopes
around: `cv_bridge` conversion, debug image setup, `set_image` (preprocessing), `predict` (GPU launch +
sync, postprocessing), each `publish_*`, and the time between callbacks), with the camera at 60 FPS
(`ros2 param set /zed/zed_node general.pub_frame_rate 60.0` at runtime).
Profiling was prepared but not run yet (the robot's battery died).

Candidates already visible in the code:

- `DebugImage::set_image` clones the full frame on every callback even when the debug image is disabled.
- `cv_bridge::toCvCopy(..., "bgr8")` converts the ZED's BGRA frame in an extra pass before preprocessing,
  which then resizes, converts the color order, normalizes and splits channels in separate OpenCV passes.
  Fusing this into one pass (or doing it on the GPU, e.g. as part of the TensorRT network) could save
  most of the preprocessing time.
- `postprocess_line_mask_rfdetr` computes the sigmoid at mask resolution and then resizes the float mask to
  full image resolution; thresholding on logits (`logit > log(t / (1 - t))`) before resizing avoids the
  `exp` and could resize a `uint8` mask instead.
- The line mask is published as a full-resolution image every frame.

### 2. Pipeline CPU and GPU work

Preprocess frame N+1 while the GPU runs frame N (two sets of buffers / two CUDA graphs).
This hides the CPU work behind the inference and should approach the network-only limit,
at the cost of one frame of added latency. Needs an asynchronous handler interface
(e.g. submit/collect) instead of `set_image` + `predict`. Decide whether the latency is acceptable.

### 3. Rewrite the line segmentation head's blocks for TensorRT

The line segmentation head is a significant part of the GPU time. Each block does
`dwconv -> Transpose(NCHW->NHWC) -> LayerNorm(channels) -> MatMul -> GELU -> Transpose(NHWC->NCHW) -> Add`,
and the transposes plus the format conversions around them are pure memory traffic on large feature maps.
Rewriting this in `onnx_layernorm_decomposition.cpp` (or a sibling rewrite) to
`dwconv -> LayerNorm over axis 1 -> 1x1 Conv (transposed MatMul weights) -> GELU -> Add` in NCHW removes them.
Validate with the same accuracy comparison as below.

### 4. Reaching 60 FPS likely needs more than the above

Even with zero overhead the current network limits the rate to about 53 FPS; most of the GPU time is
spent in the DINOv2 backbone, which TensorRT already fuses into one Myelin node. Options:

- INT8 post-training quantization, calibrated on camera frames from the recordings in `~/rosbags` on the
  robots (`IInt8EntropyCalibrator2`). Gain for the transformer layers on TensorRT 8.5 is uncertain;
  measure the speed first with an uncalibrated `trtexec --int8` build before investing in calibration.
- A model exported at a smaller input resolution or a smaller RF-DETR variant (requires ML work).

## How the experiments were done

The experiment files lived in `/tmp/trt_exp` on Peter and are likely lost after the reboot. To redo them:

- **Decomposed ONNX for `trtexec`:** a small tool calling `onnx_rewrite::decompose_layer_normalization`
  on `rfdetr.onnx` and writing the result to a file.
- **Precision variants:** `trtexec --layerPrecisions` did **not** take effect for these layers, so a small
  standalone builder using the TensorRT API (same calls as `TensorRtHandler::build_or_load_engine`) was used
  to pin chosen decomposed layers to FP32. It compiles with the system `g++` on the robot:
  `g++ -std=c++17 tool.cpp -I/usr/local/cuda/include -lnvinfer -lnvonnxparser`.
- **Timing:** `/usr/src/tensorrt/bin/trtexec --loadEngine=<engine> --useCudaGraph`; per-layer profile with
  `--dumpProfile --separateProfileRun --exportProfile=<file>.json`.
- **Accuracy:** frames extracted from the game recording in `~/rosbags` with `rosbag2_py`, preprocessed like
  `preprocess_image_rfdetr`, run through ONNX Runtime (reference) and through the engines via
  `trtexec --loadInputs=input:<file> --exportOutput=<file>.json`, then compared after the node's
  postprocessing (per-class softmax thresholds, box matching by IoU, line mask IoU).
- After changing engine build settings, increment `kEngineBuildRevision` in `tensorrt_handler.cpp` so the
  cached engines on the robots are rebuilt.
