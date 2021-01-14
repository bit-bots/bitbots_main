=======
BitFoot
=======

The BitFoot is the foot pressure sensor by the Hamburg Bit-Bots.
It features a higher update rate and higher resolution than the Rhoban ForceFoot_ on which it is based.

This board measures four differential voltage signals from load cells. It connects to a RS485 or TTL Bus compatible with Dynamixel motors from Robotis.

We managed to achieve a sensor update rate of 697Hz. The board itself can be read faster than 1kHz from the Dynamixel bus.

Firmware, schematics, gerber files, and BOMs can be found in our git repository_:

In a previous version, we used an STM32F103 microcontroller on a BluePill board.
Since we had some issues with the microcontrollers performance we switched to an ESP32.

Because we did not want to redesign the analog part of the board, we designed an adapter board from the ESP32 to the BluePill pinout.

.. _ForceFoot: https://www.github.com/Rhoban/ForceFoot
.. _repository: https://www.github.com/bit-bots/bit_foot

Flashing the Device
===================

The software is written using the Arduino framework.
To compile the software, two libraries and the ESP32 toolchain for Arduino are required.


The toolchain can be installed using the Boards Manager in the `Arduino IDE <https://www.arduino.cc/en/software>`_.

Libraries to be installed in the libraries folder of the Arduino IDE (normally "~/Arduino/libraries" on Linux):

ADS126X_

Dynamixel2Arduino_


.. _ADS126X: https://github.com/Molorius/ADS126X
.. _Dynamixel2Arduino: https://github.com/ROBOTIS-GIT/Dynamixel2Arduino

After setting the ESP32 Dev Module as the board, you should be able to compile the software.

For flashing we recommend to use a `Programmer Tool <https://www.aliexpress.com/i/32980686343.html>`_
(like this one) before soldering. After soldering there is no connector and U1 on the connector board needs to be desoldered.

With the Programmer Tool the flashing of the device should be simple with the Arduino IDE.

Strain Gauge Connection
=======================

P1: Back Right P2: Back Left P3: Front Right P4: Front Left

Calibrating the Sensors
=======================

After the device has been connected to the DXL Bus, launch the hardware interface:

:code:`roslaunch bitbots_ros_control ros_control_standalone.launch only_pressure:=true`

Then run the calibration node:

:code:`rosrun bitbots_ros_control pressure_calibration.py`

The node will guide you through the process of calibrating the cleats.
