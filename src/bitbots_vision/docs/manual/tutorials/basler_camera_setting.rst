===============
Camera Settings
===============

In the case of bad or different lighting situations (e.g at an exhibition) the camera settings need to be adjusted.
Our robots are using a Basler ace camera in the ROS environment most of the time, so this tutorial focuses on this specific configuration.

.. warning::

   All robots have individual cameras!
   Do not switch the camera without changing the name of the calibration file and the name of the camera itself in the Pylon GUI.
   Otherwise, the robot will not connect to the camera, which results in a blind robot.

   This is implemented to fix cross over issues related to two robots being in the same network and both using their cameras over ethernet.

Basler Camera Settings
----------------------

The settings for the basler cameras are located in the file ``bitbots_misc/bitbots_basler_camera/config/camera_settings.yaml``.

Only the following parameters need to be adjusted.
Other parameter values are mostly related to the ethernet connection of the camera.
These should not be adjusted!
Even if some frames get dropped with an error message.
As long, as there are frames left, everything is okay.

The interesting parameters are:

- *frame_rate*: This is kept at 10 FPS due to vision runtime limitations, but could be increased if needed.
- *exposure*: The exposure determines how long each image is exposed.
  If you increase this value, the image gets brighter, but it is also more impacted by motion blur.
  It is good to keep this value low to keep the image nice sharp.
- *gain*: The gain sets the sensitivity of the sensor.
  A low gain results in a darker, but less noisy and more color-accurate image.
  Lower values are better, as long, as the image is bright enough.

The main issue is to keep motion blur, noise and color errors low, while maintaining enough brightness to operate.

So, if the image is too dark, turn up the exposure.
If too much motion blur occurs, turn it a bit down and turn up the gain until too much noise occurs.
You need to balance the exposure and gain a bit to trade-off blur and noise while keeping the correct brightness.

If the image is too bright, turn down gain and exposure.
Here you also need to trade off the different errors.

Sadly the parameters can not be dynamically reconfigured.
So, you need to restart the driver, every time you change something.
To speed this a bit up, you can start the driver without the vision by typing

``ros2 launch bitbots_basler_camera basler_camera.launch``.

Make sure no other driver instances are running in the background.


Camera Calibration
------------------

The camera calibration is loaded automatically depending on the robot's name. If you connect a camera locally a default calibration is loaded.

The calibration files names are following the pattern ``bitbots_misc/bitbots_basler_camera/config/camera_calibration_<name>.yaml``.

To create new calibration files see

.. _`ROS camera calibration`: https://navigation.ros.org/tutorials/docs/camera_calibration.html
