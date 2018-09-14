README
======

This is the vision package of the Hamburg Bit-Bots.
We use the wolves_image_provider_v4l package as image provider.
Settings considering the vision are set in the visionparams.yaml
(bitbots_vision/config/visionparams.yaml). To be able to test new parameters,
the debug_visionparams.yaml are provided. These are used when the vision is
started via the bitbots_debug_vision launchscript.
You do NOT want to enable DEBUG on the robot.
The color calibration files are created with the wolves colorpicker and a
rosbag.
The source code of the vision is located in bitbots_vision/scripts.
In bitbots_vision/models the tensorflow classifier models are stored. Due to
their size, these are not part of this repository.
To tweak the camera image, use the settings in the image provider.
WARNING: uvcdynctrl is required by the image provider. Make sure it is installed

Required Packages
-----------------

**pip** *pip install PACKAGE*
- tensorflow
- opencv-python

**apt** *sudo apt install PACKAGE*
- uvcdynctrl

Launchscripts
-------------

To start the vision, use 
```
roslaunch bitbots_vision_common vision_startup.launch
```

```debug:=true``` publishes a debug-image which can be inspected in the rqt image view
```sim:=true``` does not start the camera (for ROS-bags or simulator)
```dummyball:=true``` does not start the ball detection to save re
 
**bitbots_vision**
- *bitbots_vision*: starts the vision with the main configuration loaded
    - Params: debug [true/FALSE], classifier [FCNN/dummy/cascade]
    
**bitbots_vision_common**
- *vision_startup*: starts the vision and the camera image provider
    - Params: debug [true/FALSE], sim [true/FALSE], dummyball [true/FALSE]
- *bitbots_vision_recordbag*: records a rosbag containing the camera image as
image_raw and stores it as /tmp/tmpbag.bag

Neural Network Models
---------------------

Currently, the models of our neural networks are not available publicly.
Due to their size, they are not included in the repository.
Bit-Bots find them in the Mafiasi NextCloud `robocup-ai/log/`
