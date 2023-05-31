YOEO-based Vision (May 2023)
============================

Description
-----------

Since 2022, the Bit-Bots vision relies on a Convolutional Neural Network called `YOEO
<https://github.com/bit-bots/YOEO>`__. Currently, YOEO provides object detection for balls and robots and semantic
segmentations for field lines, the field itself and the background. Furthermore, it is able to discriminate between teammates and opponents in case of the robot class. Goal posts are supported code-wise, but not actively used at the moment.

Supported Neural Network Frameworks
-----------------------------------
YOEOs can be run with the following frameworks: PyTorch, Onnx, OpenVino and TVM.

How to Add a New YOEO Variant to the Vision?
--------------------------------------------
In general, all models are stored in the ``models`` subdirectory. Therein, each YOEO variant comes with a dedicated
subdirectory containing a file ``model_config.yaml`` as well as subdirectories named after the supported frameworks. The latter contain the actual model files for each framework. There is no need to create framework subdirectories (and the
corresponding model files) for frameworks you do not plan to use.

A full directory structure for a YOEO variant with the name "my_model" would look like this:

::

 models/
     my_model/
         openvino/
             yoeo.bin
             yoeo.xml
         onnx/
             yoeo.onnx
         pytorch/
             yoeo.cfg
             yoeo.pth
         tvm/
             yoeo.json
             yoeo.params
             yoeo.so
         model_config.yaml
As the name already implies, the ``model_config.yaml`` provides information about the YOEO configuration. These include
the supported classes (both for object detection and semantic segmentation) as well as whether the YOEO
does provide team color detection, i.e. is able to discriminate between teammates and opponents. The file structure
looks like this:

::

    detection:
        classes:
            - ball
            - robot_red
            - robot_blue
            - robot_unknown
    team_colors: true

    segmentation:
        classes:
            - background
            - lines
            - field

Of course, the actual class names need to be adapted to your specific YOEO variant. The following names are supported:

* Object detection: 'ball', 'goalpost', 'robot', 'robot_red', 'robot_blue', 'robot_unknown'
* Semantic segmentation: 'background', 'lines', 'field'

If 'team_colors' is set to true, the robot class has to be split into 'robot_red', 'robot_blue' and 'robot_unknown'. In
all other cases, a single 'robot' class is expected, and the team color detection will automatically fall back to the HSV
detectors used in the legacy, non-YOEO-based vision.

How to Add New Classes?
-----------------------
Adding a new class requires some work, but should be a relatively straightforward endeavor. The necessary steps are as
follows:

#. Add the new class to the ``model_config.yaml`` of your YOEO variant. Do not forget, your YOEO variant must support this class too!
#. Create a detector class for your new class. A favorable location for this would be ``bitbots_vision/vision_modules/yoeo/detectors.py``.
#. Create a vision component for your new class. A favorable location for this would be ``bitbots_vision/vision_modules/yoeo/vision_components.py``.
#. Add your newly created component to the import statement in ``bitbots_vision/vision_components/yoeo/__init__.py``.
#. Instantiate and add your newly created component to the vision pipeline in ``bitbots_vision/yoeo_vision.py``.
#. Optional but highly recommended: add a parameter to ``config/yoeo_vision_params.yaml``, ``config/yoeo_visionparams_sim.yaml`` and ``bitbots_vision/yoeo_params.py`` which allows for activating / deactivating your new component dynamically.

Code structure
--------------
The class ``YOEOVision`` serves as the entry point for the vision. It is responsible  for configuring and running the entire vision pipeline.

The vision pipeline is modularized, i.e. each (core) functionality is provided by a dedicated component, for example
the ``LineDetectionComponent`` for the line segmentation, the ``BallDetectionComponent`` for the ball detection, etc. Each
component is solely responsible for providing its functionality, i.e. once it receives an image, it carries out all steps required to fulfill its functionality, and forwards its result (if necessary). Apart from the YOEOComponent, which
runs the YOEO instance and is thus a 'must-have' component, all other vision components are optional and can be
activated or deactivated at any time.

One level further down are the YOEO components. These components are basically just a bunch of wrapper classes that
take an ``IYOEOHandler`` as constructor argument and provide a single class from the YOEO output. The idea behind these wrappers is to have as few classes as possible depend on the actual class names (which are provided by the
``model_config.yaml``).

Last but not least, there are the YOEO handlers which implement the aforementioned ``IYOEOHandler`` interface. The handlers
are responsible for actually running the YOEO network: from pre-processing the input, over feeding the pre-processed
input into the neural network, to post-processing the network output. Hence, there is exactly one handler per supported
framework.
