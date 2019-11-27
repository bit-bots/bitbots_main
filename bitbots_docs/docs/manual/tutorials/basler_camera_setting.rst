===============
Camera Settings
===============

In the case of bad or different lighting situations (e.g at an exhibition) the camera settings need to be adjusted.
Our robots are using a Basler ace in the ROS environment most of the time, so this tutorial focuses on this specific configuration.

.. warning::

   All robots have individual cameras!
   Do not switch the camera without changing the name of the calibration file and the name of the camera itself in the Pylon GUI.
   Otherwise, the robot will not connect to the camera, which results in a blind robot.

   This is implemented to fix cross over issues related to two robots being in the same network and both using their cameras over ethernet.

Basler Camera Settings
----------------------

The settings for the basler camera are located in the file ``basler_drivers/pylon_camera/config/camera_settings.yaml``.

Only the following parameters need to be adjusted. Other parameter values are mostly related to the ethernet connection of the camera.
These should not be adjusted! Even if some frames get dropped with an error message. As long as there are frames left everything is okay.

The interesting parameters are:

- *frame_rate*: This is kept at 10 FPS due to vision runtime limitations but could be increased if needed.
- *exposure*: The exposure determines how long each image is exposed.
  If you increase the value the image gets brighter but it is also more impacted by motion blur.
  It is good to keep this value low to keep the image nice sharp.
- *gain*: The gain sets the sensitivity of the sensor. A low gain results in a darker, but less noisy and more color-accurate image.
  Lower values are better as long as the image is bright enough.

The main issue is to keep motion blur, noise and color errors low while maintaining enough brightness to operate.

So if the image is too dark, turn up the exposure. If too much motion blur occurs, turn it a bit down and turn up the gain until too much noise occurs.
You need to balance the exposure and gain a bit to trade-off blur and noise while keeping the correct brightness.

If the image is too bright, turn down gain and exposure. Here you also need to trade off the different errors.

Sadly the parameters can not be dynamically reconfigured. So you need to restart the driver every time you change something.
To speed this a bit up, you can start the driver without the vision by typing

``roslaunch bitbots_bringup basler_camera.launch``.

Make sure no other driver instances are running in the background.


Camera Calibration
------------------

The camera calibration is loaded automatically depending on the robot's name. If you connect a camera locally a default calibration is loaded.

The calibration files names are following the pattern ``basler_drivers/pylon_camera/config/camera_calibration_<name>.yaml``.

To create new calibration files see

.. _`ROS camera calibration`: http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

White balance
-------------

In case of a correct brightness, but wrong looking colors have a look at the following part.

This part should be tweaked after all brightness and gain settings because they could possibly affect the color temperature too.

Our vision pipeline includes a white balancing node on the ROS level.
Cameras need to adjust their white balance to correct the effects of different light temperatures.
E.g. a light bulb casts a different light color than the sun.
Most cameras do this automatically, but wrong values could result in unusable images for the vision.
Therefore we set this value manually.

The light temperature can be set in the dynamic reconfigure UI of the *whitebalancer*.
This should be used to find the best settings by tweaking the parameter a bit until the colors nearly look like the colors in real life.

A good benchmark should be white objects in the image, which should be displayed white.

Note: Sometimes human vision recognizes slightly blue-white tones whiter than the real white.
So if the object appears unnaturally white, turn the setting a bit more in the warmer direction even if the white looks a bit grayish now. This is a brightness issue.


After tweaking the settings the estimated value can be written in the following config file under the name *temp*.

``bitbots_vision/white_balancer/config/config.yaml``

Some good indoor value should be something like 3000 K and for outdoor applications something about 6500 K.
