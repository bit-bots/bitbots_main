===========
Colorpicker
===========

How to Record a Color Lookup Table
===========================

Robot Camera
------------
1. Connect your PC to the Jetson in the robot.
2. Run ``roslaunch bitbots_bringup basler_camera.launch`` on the robot.
3. Check if too many images are dropped. If this is the case, check your MTU size.
4. Set ``ROS_IP`` and ``ROS_MASTER_URI`` in your local ros environment. ``export ROS_MASTER_URI=http://<Robot_NUC_IP>:11311/`` and ``export ROS_IP=<your_IP>``
5. Run ``rosrun bitbots_vision colorpicker.py`` to run the colorpicker on your PC.
6. Now select all field colors, excluding the lines by clicking on them in the appearing window. For further usage look at the terminal window.


External Camera
---------------
1. Connect your PC with the Camera using a POE adapter.
2. Run ``roslaunch bitbots_bringup basler_camera.launch`` in your local ROS environment.
3. Check if too many images are dropped. If this is the case, check your MTU size.
4. Run ``rosrun bitbots_vision colorpicker.py`` to run the colorpicker on your PC.
5. Now select all field colors, excluding the lines by clicking on them in the appearing window. For further usage look at the terminal window.

If you use another generic USB camera, make sure it's driver publishes to ``/camera/image_proc`` or use the ``wolves_image_provider`` (See https://github.com/bit-bots/wolves_image_provider).

What's next?
============
1. Move the just saved ``.pickle`` file to ``bitbots_meta/bitbots_vision/bitbots_vision/config/color_lookup_tables``.
2. Find a usefull name. Normally the files are names like that: ``<camera_type>_<room_or_location>_<special_attributes>.pickle``.
3. Select the color lookup table via dynamic reconfigure or change the parameter in the visionparams.yaml.
4. Win the RoboCup.
