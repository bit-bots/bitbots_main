README
======

This is the vision package of the Hamburg Bit-Bots.
For standardized camers such as USB-webcams we use the wolves_image_provider package as image provider.
Alternatively every image source, that publishes sensor_msgs/Image messages (i.e. basler driver for basler cameras) is supported.
Settings considering the vision are set in the visionparams.yaml
(bitbots_vision/config/visionparams.yaml).
You do NOT want to enable DEBUG on the robot.
The color calibration files are created with the wolves colorpicker and a
rosbag.
The source code of the vision is located in bitbots_vision/src.
In bitbots_vision/models the tensorflow classifier and fcnn models are stored. Due to
their size, these are not part of this repository.
To tweak the camera image, use the settings in the image provider.

Neural Network Models
---------------------

Currently, the models of our neural networks are not available publicly.
Due to their size, they are not included in the repository.
Bit-Bots find them in the Mafiasi NextCloud `robocup-ai/log/`


Vision Tools
------------

In the bitbots_vision_tools directory, special tools for debugging/introspection purposes are provided.


Launchscripts
-------------

To start the vision, use 
```
roslaunch bitbots_vision vision_startup.launch
```

```sim:=true``` sets the param ```ùse_sim_time``` true and does not start a image provider(for ROS-bags or simulator)
```dummyball:=true``` does not start the ball detection to save resouces
```debug:=true``` publishes a debug-image which can be inspected in the rqt image view
```use_game_settings:=true``` does load additional game settings
```basler:=false```does start wolves image provider instead of the basler camera driver
 
**bitbots_vision**
- *vision_startup*: starts the vision and a camera image provider
    - Params: 
        - debug [true/FALSE], 
        - sim [true/FALSE],
        - dummyball [true/FALSE],
        - use_game_settings [true/FALSE],
        - basler [TRUE/false]

**bitbots_vision_tools**
- *bitbots_vision_recordbag*: records a rosbag containing the camera image as image_raw and stores it as /tmp/tmpbag.bag
