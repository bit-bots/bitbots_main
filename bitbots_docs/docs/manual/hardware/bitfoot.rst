=======
BitFoot
=======

The BitFoot is the foot pressure sensor by the Hamburg Bit-Bots.
It features a higher update rate and theoretically higher precision than the `Rhoban ForceFoot <https://www.github.com/Rhoban/ForceFoot>`_ on which it is based.

Firmware, schematics and instructions how to flash firmware can be found in the git repository_:


.. _ForceFoot: https://www.github.com/Rhoban/ForceFoot
.. _repository: https://www.github.com/bit-bots/bit_foot



Calibrating the Sensors
=======================

After the device has been connected to the DXL Bus, launch the hardware interface:

:code:`roslaunch bitbots_ros_control ros_control_standalone.launch only_pressure:=true`

Then run the calibration node:

:code:`rosrun bitbots_ros_control pressure_calibration.py`

The node will guide you through the process of calibrating the cleats.
