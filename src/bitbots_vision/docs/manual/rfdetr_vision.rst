RF-DETR-based Vision
=====================

Description
-----------

The Bit-Bots vision uses `RF-DETR <https://github.com/roboflow/rf-detr>`__, a transformer-based, NMS-free
object detector, for detecting balls, robots, and goal posts. Unlike the previous YOEO-based pipeline, RF-DETR
has no segmentation head, so field lines are instead detected with a classical HSV-based color segmentation
step: the field is treated as the (roughly convex, from a downward/forward-facing robot camera) region
enclosed by the green turf's outer boundary, and a candidate white pixel only counts as a line pixel if it
falls inside that filled-in field region -- i.e. lines, the ball, and robots standing on the field are "holes"
in the field's convex hull. This avoids classifying white background clutter (e.g. crowd, advertising boards)
as a line, since such clutter normally lies outside the field's boundary rather than inside it. Any region
covered by a detected ball, robot, or goal post is additionally painted out of the line mask.

Since a convex hull can bulge slightly past the true (possibly non-convex) field boundary at concave points,
the hull is eroded inward by a configurable margin (``line.field_border_margin_px``) before being used, trading
a thin strip of near-boundary line detection for suppressing artifacts from background just outside the real
edge. In the debug image, the field region is drawn as a diagonal hatch pattern (rather than a solid tint) in a
color distinct from the turf itself, so it stays visually distinguishable from both the raw video and other
overlays.

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
        - unknown
        - goalpost
        - robot

The RF-DETR model currently in use has no team-color information, so all detected robots are published with
an "unknown" team. A class name that the vision node never queries by name (e.g. ``unknown`` above) acts as a
harmless background/no-object sink — candidates classified into it simply accumulate internally and are never
published.

Code structure
--------------

* ``RfdetrHandler`` (``rfdetr_handler.hpp``/``.cpp``) wraps the ONNX Runtime session: execution-provider
  selection (TensorRT → CUDA → WebGPU → CPU fallback chain), model I/O introspection, and running inference.
* ``rfdetr_processing.hpp``/``.cpp`` contains the pure (ONNX-free, unit-tested) preprocessing and
  postprocessing math: direct resize + ImageNet normalization on the way in, softmax + confidence threshold +
  box decoding on the way out. RF-DETR is NMS-free, so there is no suppression step.
* ``line_segmentation.hpp``/``.cpp`` contains the pure HSV line-segmentation pipeline: white/green HSV
  thresholding, the field convex-hull fill, basic morphological/contour-area filtering, and
  detected-object suppression.
* ``VisionNode`` (``vision_node.cpp``) wires the above together and publishes ROS messages.
