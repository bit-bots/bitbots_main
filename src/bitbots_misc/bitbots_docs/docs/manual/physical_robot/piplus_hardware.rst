PiPlus Hardware Overview
========================

This page gives an overview of the hardware of the PiPlus robot platform.

.. todo::
   Rework of the public documentation (see issue #1037): write the PiPlus
   hardware overview. It should replace the removed Wolfgang electronics and
   mechanics pages and cover the following:

   Motors & Joints
      Which motors and joints does the PiPlus have? Derive the joint names and
      motor IDs from the livelybot serial configuration
      (``src/lib/livelybot_hardware_sdk/src/livelybot_serial/config/robot.yaml``)
      and the description package (``piplus_description``). This replaces the
      former Wolfgang "servo numbers" reference.

   Computation devices
      Which computation devices are on the robot (e.g. main computer, cameras)?

   Sensors
      Which sensors are available (e.g. the IMU built into the livelybot serial
      protocol, the camera)?

   Other outputs
      Which other outputs exist? Document the built-in speaker and the LCD
      display. This replaces the former standalone "Speaker" page.

   Ports
      Which ports are there? Document the USB ports and which is which.
