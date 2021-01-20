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

Software
========

Firmware
--------

The firmware is uses the Arduino framework. We usually install it using the Arduino IDE.
The required libraries are:

* `ADS126X <https://github.com/Molorius/ADS126X>`_
* `Dynamixel2Arduino <https://github.com/ROBOTIS-GIT/Dynamixel2Arduino>`_

For installing the build tools for the ESP32 refer to `espressif's documentation <https://github.com/espressif/arduino-esp32#installation-instructions>`_.

For flashing a ESP32 Wroom (without development board) we recommend a `programming socket <https://www.aliexpress.com/i/32980686343.html>`_.

Flash the board before soldering!

ROS Control
-----------

We developed a hardware interface that complies with the ros_control standard for the Wolfgang robot platform.
This includes a hardware interface for the BitFoot. It can be found `here <https://github.com/bit-bots/bitbots_lowlevel/tree/master/bitbots_ros_control>`_.

Strain Gauge Connection
=======================

The strain gages should be connected as follows when using our ros_control based software:

* P1: Back Right
* P2: Back Left
* P3: Front Right
* P4: Front Left


.. _Calibrating the Sensors:

Calibrating the Sensors
=======================

After the device has been connected to the DXL Bus, launch the hardware interface:

:code:`roslaunch bitbots_ros_control ros_control_standalone.launch only_pressure:=true`

Then run the calibration node:

:code:`rosrun bitbots_ros_control pressure_calibration.py`

The node will guide you through the process of calibrating the cleats.

Register Table
==============

+--------+--------+--------------------------+--------+---------+---------+-------------+
| Adress | Length | Name                     | Access | Default | Type    | Persistent? |
+========+========+==========================+========+=========+=========+=============+
| 7      | 1      | :ref:`id<DXL>`           | rw     | 101     | int8    | yes         |
+--------+--------+--------------------------+--------+---------+---------+-------------+
| 8      | 1      | :ref:`baud<DXL>`         | rw     | 4       | int8    | yes         |
+--------+--------+--------------------------+--------+---------+---------+-------------+
| 36     | 4      | :ref:`sensor_0<Sensors>` | r      |         | float32 |             |
+--------+--------+--------------------------+--------+---------+---------+-------------+
| 40     | 4      | :ref:`sensor_1<Sensors>` | r      |         | float32 |             |
+--------+--------+--------------------------+--------+---------+---------+-------------+
| 44     | 4      | :ref:`sensor_2<Sensors>` | r      |         | float32 |             |
+--------+--------+--------------------------+--------+---------+---------+-------------+
| 48     | 4      | :ref:`sensor_3<Sensors>` | r      |         | float32 |             |
+--------+--------+--------------------------+--------+---------+---------+-------------+

.. _DXL:

DXL
---

**id**: Can be a value between 1 and 252. it is used to talk to the device over the Dynamixel bus.

**baud**: Can be a value between 0 and 7

+-------+---------+--------+
| value | baud    | Tested |
+=======+=========+========+
| 0     | 9,600   | no     |
+-------+---------+--------+
| 1     | 57,600  | no     |
+-------+---------+--------+
| 2     | 115,200 | no     |
+-------+---------+--------+
| 3     | 1M      | no     |
+-------+---------+--------+
| 4     | 2M      | yes    |
+-------+---------+--------+
| 5     | 3M      | no     |
+-------+---------+--------+
| 6     | 4M      | yes    |
+-------+---------+--------+
| 7     | 4.5M    | no     |
+-------+---------+--------+

We are reasonably certain that the other baud rates work as well since the ESP32 supports them.

.. _Sensors:

Sensors
-------

**sensor_{0..3}**: Raw reading of the sensors differential voltage. Must be :ref:`calibrated<Calibrating the Sensors>` to give a meaningful reading.

* sensor_0 = P4
* sensor_1 = P3
* sensor_2 = P2
* sensor_3 = P1
