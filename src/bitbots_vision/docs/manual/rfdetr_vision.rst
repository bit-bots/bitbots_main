RF-DETR-based Vision
=====================

Description
-----------

The Bit-Bots vision uses `RF-DETR <https://github.com/roboflow/rf-detr>`__, a transformer-based, NMS-free
object detector, for detecting balls, goal posts, obstacles, and robots. Unlike the previous YOEO-based
pipeline, the object-detection and line-segmentation heads are two separate outputs of the same RF-DETR model
rather than a single combined network: field lines come from the model's own learned line-segmentation head,
not a classical color-based heuristic. The head outputs a low-resolution probability map, which is resized to
the original image resolution and thresholded to produce the line mask. Any region covered by a detected ball,
robot, or goal post is additionally painted out of the line mask as a safety net against the segmentation head
misclassifying part of an object as a line.

FP16 model exports
-------------------

RF-DETR ONNX exports are available with either fp32 or fp16 internal weights; both keep the same float32
input/output tensor contract (only internal weights differ), so no ``RfdetrHandler`` code needs to change to
use one -- just place whichever variant's ``.onnx`` file you want at ``<model_dir>/onnx/rfdetr.onnx``. (This
does not apply to ``TensorRtHandler``, which always builds its engine from the fp32 export and lets TensorRT
decide FP16 usage per layer -- see below.)

Inference backends
-------------------

Two inference backends implement the same ``RfdetrHandlerInterface``, and ``VisionNode`` uses whichever one
was compiled in:

* ``RfdetrHandler`` (``rfdetr_handler.hpp``/``.cpp``) -- ONNX Runtime, always built. Execution providers are
  tried in priority order: TensorRT → CUDA → WebGPU → CPU (always available as fallback). This is the portable
  backend used on the x86 dev environment and on any robot without a build-time-detected TensorRT install.
* ``TensorRtHandler`` (``tensorrt_handler.hpp``/``.cpp``) -- native TensorRT (``nvinfer1``/``nvonnxparser``),
  bypassing ONNX Runtime entirely. Only compiled in, and preferred at runtime, when TensorRT + CUDA are found
  at build time (see "TensorRT backend" below). Targets JetPack 5.1.5 (TensorRT 8.5.2 / CUDA 11.4), but only
  uses TensorRT APIs stable since TensorRT 8.0.

Both backends share the same pure, unit-tested pre/postprocessing math in ``rfdetr_processing.hpp``/``.cpp``
(direct resize + ImageNet normalization in; softmax + per-class confidence threshold + box decoding for
detections; sigmoid + resize + threshold for the line mask; ``suppress_candidates`` for painting
detected-object regions out of the line mask) -- only model I/O plumbing differs between them. RF-DETR is
NMS-free, so neither backend runs NMS.

TensorRT backend
~~~~~~~~~~~~~~~~~

``CMakeLists.txt`` searches for TensorRT (``NvInfer.h`` / ``libnvinfer.so`` / ``libnvonnxparser.so``) and CUDA
(``cuda_runtime_api.h`` / ``libcudart.so``) the same way it searches for ONNX Runtime -- a manual
``find_path``/``find_library``, since neither ships an official CMake config package on JetPack. If found,
``TensorRtHandler`` is compiled in and ``VisionNode`` prefers it unconditionally; if not (e.g. the x86 dev
environment), only ``RfdetrHandler`` is built and nothing else about the build changes.

On first construction for a given model directory, ``TensorRtHandler`` builds a TensorRT engine from
``<model_dir>/onnx/rfdetr.onnx`` and caches it to ``<model_dir>/onnx/rfdetr.trt`` -- this can take several
minutes on a Jetson. Subsequent runs load the cached engine directly (a few seconds); the cache is rebuilt
automatically if the ``.onnx`` file is newer than the cached engine. A TensorRT engine is tied to the exact
GPU + TensorRT version it was built on -- delete the cached ``.trt`` file to force a rebuild after a
TensorRT/driver upgrade or a GPU swap. FP16 is enabled automatically whenever the platform supports it
(``IBuilder::platformHasFastFp16()``, true on JetPack 5.1.5 Xavier/Orin targets); TensorRT decides per-layer
whether FP16 or FP32 is used.

This backend was written and code-reviewed without access to TensorRT/CUDA hardware (not available in the dev
environment) -- it has not been build- or runtime-tested. If it doesn't build or misbehaves on the robot, the
error messages are logged verbosely (including ONNX parser errors and CUDA error strings) to aid debugging;
falling back to deleting the TensorRT include/library detection results (or simply not having TensorRT
installed) always leaves the ONNX Runtime backend as a working fallback.

Model directory layout
-----------------------

The model directory (pointed at by the ``model.path`` parameter) must contain:

::

 <model_dir>/
     onnx/
         rfdetr.onnx
     model_config.yaml

``model_config.yaml`` lists the detection classes in the exact order matching the model's output channels:

::

    detection:
      classes:
        - ball
        - goalpost
        - obstacle
        - robot
        - unknown

The RF-DETR model currently in use has no team-color information, so all detected robots are published with
an "unknown" team. A class name that the vision node never queries by name (e.g. ``unknown`` above, the
model's background/no-object sink) simply accumulates internally and is never published. Detected obstacles
are likewise not currently published by ``VisionNode`` -- the class exists in the model but has no wired-up
topic yet.

Code structure
--------------

* ``RfdetrHandlerInterface`` (``rfdetr_handler_interface.hpp``) is the common interface both backends
  implement; ``RfdetrConfig`` (``rfdetr_config.hpp``) is the config type shared by both.
* ``RfdetrHandler`` / ``TensorRtHandler`` -- see "Inference backends" above.
* ``rfdetr_processing.hpp``/``.cpp`` -- shared pure pre/postprocessing math, see above.
* ``VisionNode`` (``vision_node.cpp``) wires the above together (choosing the backend via
  ``#ifdef BITBOTS_VISION_HAS_TENSORRT``) and publishes ROS messages.
