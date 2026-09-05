Bitbots Lowlevel
================

.. todo::
   Rework of the public documentation (see issue #1037): this page previously
   described the Wolfgang low level stack (Dynamixel motors on RS-485/TTL, the
   CORE board, the Dynamixel SDK/Workbench and ``bitbots_ros_control``). None of
   that applies to the PiPlus platform, which uses livelybot CAN-FD actuators
   over a serial link.

   Rewrite this page to describe the PiPlus low level stack:

   - What the low level packages do and how the control loop is structured.
   - The livelybot hardware SDK and serial protocol
     (``src/lib/livelybot_hardware_sdk``) and how motors and the built-in IMU are
     read and written.
   - How the hardware interface integrates with ROS 2 control.
   - Common problems and troubleshooting strategies.

   See :doc:`piplus_hardware` for the hardware overview and
   :doc:`test_robot_hardware` for hardware/lowlevel testing.
