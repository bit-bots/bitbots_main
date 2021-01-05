===========
Colorpicker
===========


The colorpicker is a tool to record a color lookup tables e.g. of field colors for later use in the vision.
It subscribes to the ``Image`` topic ``/camera/image_proc``.


How to Record a Color Lookup Table
===========================

1. Publish images to the topic ``/camera/image_proc``:

  * Using a **Robot Camera**:

    #. Connect your PC to the nuc in the robot. (See in internal documentation for more details)
    #. Set ``ROS_IP`` and ``ROS_MASTER_URI`` in your local ros environment. ``export ROS_MASTER_URI=http://<Robot_NUC_IP_OR_HOSTNAME>:11311/`` and ``export ROS_IP=<your_IP>``
    #. Run ``roslaunch bitbots_bringup basler_camera.launch`` on the robot.

  * Using an **External Basler Camera**:

    #. Connect your PC with the Camera using a POE adapter.
    #. For the first time, you have to setup the pylon camera driver. Run ``make basler`` in the ``bitbots_meta`` directory and then ``catkin build pylon_camera`` in you catkin workspace.
    #. Start the pylon camera driver with ``roslaunch bitbots_bringup basler_camera.launch`` in your local ROS environment.
    #. Check if too many images are dropped by the pylon camera driver.
       If this is the case, check your MTU size using this guide `MTU guide <https://linuxways.net/ubuntu/how-to-change-mtu-size-in-linux/>`_
       Your MTU size should match the camera's MTU size, specified in `this config <https://git.mafiasi.de/Bit-Bots/basler_drivers/src/branch/master/pylon_camera/config/camera_settings.yaml>`_.

  * Using a **rosbag**:

    1. Check, if your rosbag contains the ``/camera/image_proc`` topic with ``rosbag info <PATH_TO_ROSBAG_FILE> | grep /camera/image_proc``

      * In case, your rosbag does not contain images under the ``/camera/image_proc`` topic:
        You can use the `rosbag remapper <https://github.com/bit-bots/bitbots_vision/blob/master/bitbots_vision/scripts/rosbag_remapper.py>`_ to update your rosbag.

    2. Play your rosbag as usual with ``rosbag play <PATH_TO_ROSBAG_FILE> --clock -l``

  * Using a another **camera**:

    You can use the ROS package `usb_cam <https://wiki.ros.org/usb_cam>`_ to publish the camera images.
    Make sure your camera's driver publishes images to the ``/camera/image_proc`` topic.

2. Run ``rosrun bitbots_vision colorpicker.py`` on your local computer to run the colorpicker.
3. Now select all field colors, excluding the lines by clicking on them in the appearing window.
   For further usage look at the terminal window.


What's next?
============
1. Move the just saved ``.pickle`` file to ``bitbots_meta/bitbots_vision/bitbots_vision/config/color_lookup_tables``.
2. Find a usefull name. Normally the files are names like that: ``<camera_type>_<room_or_location>_<special_attributes>.pickle``.
3. Select the color lookup table via dynamic reconfigure or change the parameter in the visionparams.yaml.
4. Win the RoboCup.
