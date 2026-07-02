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
input/output tensor contract (only internal weights differ), so no handler code needs to change to use one --
just place whichever variant's ``.onnx`` file you want at ``<model_dir>/onnx/rfdetr.onnx``.

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

* ``RfdetrHandler`` (``rfdetr_handler.hpp``/``.cpp``) wraps the ONNX Runtime session: execution-provider
  selection (TensorRT → CUDA → WebGPU → CPU fallback chain), model I/O introspection (outputs are resolved by
  name -- "dets", "labels", and the optional "line_mask" -- not by position, since the output set/order can
  vary between model exports), and running inference.
* ``rfdetr_processing.hpp``/``.cpp`` contains the pure (ONNX-free, unit-tested) pre/postprocessing math: direct
  resize + ImageNet normalization on the way in; softmax + per-class confidence threshold + box decoding for
  detections; sigmoid + resize + threshold for the line mask. RF-DETR is NMS-free, so there is no NMS step.
  Also contains ``suppress_candidates``, which paints detected-object regions out of the line mask.
* ``VisionNode`` (``vision_node.cpp``) wires the above together and publishes ROS messages.
