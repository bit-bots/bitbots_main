=============
Wolfgang CORE
=============

`Github repository <https://github.com/bit-bots/wolfgang_core>`_

.. image:: img/core.png
  :width: 800

Features
========


* Power over Ethernet (PoE) power source (for ethernet camera)
* Power regulation 5V ~5A (Odroid/Raspberry Pi), 9V 1A (network switch)
* Power source selection (Battery or power supply)
* Switching of power to Motors (Manual and through software)
* RS485/TTL on 4 Buses with theoretically up to 12 MBaud (4MBaud tested since thats what our Dynamixel motors support)
* Voltage (including Cell voltage for LiPos) and Current monitoring
* 3 RGB LEDS!!!


Software
========


Firmware
--------

Required Libraries:

* `Dynamixel2Arduino <https://github.com/ROBOTIS-GIT/Dynamixel2Arduino>`_
* `FastLED <https://github.com/FastLED/FastLED>`_

Installation instructions for the Teensy 4.0 for the Arduino IDE can be found `here <https://www.pjrc.com/teensy/td_download.html>`_.

ROS Control
-----------

We have written a ros_control hardware interface for the board.
You can use it or use it as a reference for your own hardware abstraction.
It can be found `in this git <https://github.com/bit-bots/bitbots_lowlevel/tree/master/bitbots_ros_control>`_.

Problems and Solutions
======================

V1.0 is unfortunately only 98% perfect.

Reading of ttyUSB1 without Teensy
---------------------------------

Devices on ttyUSB3 can not be read when the Teensy is not installed or installed and not powered (see :ref:`Jumpers`)
since the !RE/DE pins of the Teensys transceiver are not pulled down.

.. _Bodge Switch:

Automatic Poweroff-Poweron Routine
----------------------------------

When software disables the power to the motors and the user turns off and on the power manually, it should be on.
To accomplish this, the Teensy has to read the power state of the manual switch.

To achieve this the trace marked in red has to be cut (for example with an x-acto knife, make sure there is really no connection)
and a wire marked in green needs to be soldered.

.. image:: img/core_bodge.png
  :width: 800

Numbering of Dynamixel Buses
----------------------------

The number behind the name below the Molex SPOX Mini connectors is meant to show which ttyUSBX virtual device corresponds to which bus.
Due to a mixup it is wrong.

Correct version:

+-----------+----------+-----------+-------------+
| ttyUSB0   | ttyUSB1  | ttyUSB2   | ttyUSB3     |
+===========+==========+===========+=============+
| Right Leg | Left Leg | Head+IMUs | Teensy+Arms |
+-----------+----------+-----------+-------------+

Status LEDs
===========

The red round LED next to the manual switch indicated that the board is powered. If it is off even though the board is plugged in, the fuse might be blown.

The green round LED next to it indicates if the motors and other devices on the Dynamixel bus are powered.

The three RGB LEDs are set by software.


Power over Ethernet
===================

The board features a Power over Ethernet (PoE) power sourcing equipment (PSE).
To be more specific, it is a midspan, meaning it injects the power part of PoE onto the signal.
It basically works like a PoE injector which are widely available but in a much nicer form factor for a robot.

The powered device should be plugged into the RJ45 port pointing to the top and the other device into the port pointing to the bottom.

PoE works with 48V which means you should not lick it 😜.

Power Regulation
================

There are two step down converters on the board to power other devices in the robot.
The 5V regulator can theoretically provide up to 10A but we have never tested it and it might need cooling or a heatsink for that.
In our case it used to power an ODROID XU4 which normally comes with a 4A power source (which it never really uses unless there are many power hungry USB devices plugged in).

The 5V regulator also powers some of the electronics on the board such as the Teensy, the LEDs and the current sensor.

A small 9V regulator is also on the board since the network switches we use in our robot run on 9V.

9V and 5V are on the Molex Mini-Lock connector on the bottom left of the board. The pinout from left to right is:

+-----+-----+-----+-----+
| GND | GND | +5V | +9V |
+-----+-----+-----+-----+

.. _Power:

Power Connectors and Power Source Selection
===========================================

There are two connectors meant to be used for soldering a connector (e.g. Tamiya or XT90) to connect a battery and power supply.

They are located on the right above the Molex SPOX Mini connectors.
They are labeled as VBAT+ and VBAT- for the battery and VEXT+ and VEXT- for the external supply. While technically they are treated the same,
it is recommended to connect them in the correct order since only VEXT is measured and VBAT should be the same as the one connected to the balancer connector.
The batteries balancer connector should be connected to P2. Be careful when soldering P2 since it needs to be oriented correctly.
The GND pin is at the very bottom. If it is plugged in the other way, the Teensy will probably blow up 🤯.

Both, a battery and a power supply, can be safely connected at the same time.
No energy is transfered from one to the other since there is a double Schottky diode (D2 on the bottom side) between them.
This can be useful when changing batteries but keeping the robot powered on using a power supply.

While some energy is lost over D2, this simple solution has proven very robust.


Switching of power to Motors
============================

Power on the Dynamixel bus can be switched on and off using either the manual switch or through software.
If the manual switch is to the right, the power is off. If it is to the right, the power is on (if the software agrees).
The manual switch is an override, meaning when it is off, the teensy can not enable it.
If there is power on the Dynamixel bus the green round LED will be on.
If it is on when the switch is in the off position (to the right), the MOSFET Q1 is probably blown and needs replacement.
This may happen if there is a short circuit and the MOSFET is weaker than the fuse.

When the power is turned off by software, but is on by the switch,
you can flip the switch off and on and the power should be back on, provided the hardware hack for V1.0 is installed (see :ref:`Bodge Switch`).

TTL or RS485 and biasing
========================

R1-R4 and R9-R12 are used to bias the differential lines of the RS485 signal such that when no transceiver is active,
no garbage gets transmitted over the bus. These components should always be populated.

The RS485 transceivers can also speak TTL if the board is configured correctly.
To achieve this R5-R8 need to be in place. This causes the B line of the RS485 signal to be held at 2.5V.
The A line of the RS485 signal will be the TTL signal.
The transceiver can interpret the incoming signal correctly since the voltage differential needs to be +0.2v for a 1
and -0.2V for a 0.
If only RS485 is used, it is recommended to leave R5-R8 unpopulated.

.. _Jumpers:

Jumpers
=======

Two jumpers exist on the Wolfgang CORE.

P1: Enables the Teensy 4.0 to switch on and off the motor power.

P2: Enables the power supply to the Teensy 4.0. **Do not use together with a USB cable plugged into the Teensy!!**



Register Table
==============

+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| Adress | Length | Name                                  | Access | Default | Type    | Persistent? |
+========+========+=======================================+========+=========+=========+=============+
| 7      | 1      | :ref:`id<DXL>`                        | rw     | 42      | int8    | yes         |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 8      | 1      | :ref:`baud<DXL>`                      | rw     | 4       | int8    | yes         |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 10     | 4      | :ref:`led0<LEDs>`                     | rw     | 0       | int8[4] | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 14     | 4      | :ref:`led1<LEDs>`                     | rw     | 0       | int8[4] | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 18     | 4      | :ref:`led2<LEDs>`                     | rw     | 0       | int8[4] | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 22     | 1      | :ref:`teensy_led<LEDs>`               | rw     | 0       | int8    | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 23     | 1      | :ref:`power_control<Power Control>`   | rw     | 1       | int8    | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 28     | 2      | :ref:`VEXT<Voltage Sensing>`          | r      |         | int16   | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 30     | 2      | :ref:`VCC<Voltage Sensing>`           | r      |         | int16   | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 32     | 2      | :ref:`VDXL<Voltage Sensing>`          | r      |         | int16   | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 34     | 2      | :ref:`current<Current Sensing>`       | r      |         | int16   | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 36     | 1      | :ref:`manual_power_on<Power Control>` | r      |         | int8    | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 38     | 2      | :ref:`VBAT_0<Voltage Sensing>`        | r      |         | int16   | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 40     | 2      | :ref:`VBAT_1<Voltage Sensing>`        | r      |         | int16   | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 42     | 2      | :ref:`VBAT_2<Voltage Sensing>`        | r      |         | int16   | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 44     | 2      | :ref:`VBAT_3<Voltage Sensing>`        | r      |         | int16   | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 46     | 2      | :ref:`VBAT_4<Voltage Sensing>`        | r      |         | int16   | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+
| 48     | 2      | :ref:`VBAT_5<Voltage Sensing>`        | r      |         | int16   | no          |
+--------+--------+---------------------------------------+--------+---------+---------+-------------+

.. _DXL:

DXL
---

**id**: Can be a value between 1 and 252. It is used to talk to the device over the Dynamixel bus.

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

We are reasonably certain that the other baud rates work as well since the Teensy supports them.


.. _LEDs:

LEDs
----

**led{0,1,2}**: Byte order: RGB, 4th byte is ignored but reserved.


.. _Power Control:

Power Control
-------------

**power_control**: Used to turn on and off the power (0 off, 1 on). Will be overwritten by manual switch if toggled.

**manual_power_on**: Indicates whether the manual power switch is on. Requires :ref:`Bodge Switch`

.. _Voltage Sensing:

Voltage Sensing
---------------

Voltages are scaled down using a voltage divider to be read by the microcontroller.
Multiplying by the given scale factor returns the actual voltage on the voltage rail.
The scale factor is given as the conversion factor from analog reading to voltage multiplied by the factor of the voltage divider.
The factor of the voltage divider is given as (top_resistor/(top_resistor+bottom_resistor)).
It is recommended to use ±0.1% or ±1% resistors for the voltage dividers.

**VEXT**: Raw reading of the external power supply voltage. Scale factor: (3.3 / 1024) * (2.0/(2.0+10.0))

**VCC**: Raw reading of the main voltage rail. Scale factor: (3.3 / 1024) * (2.0/(2.0+10.0))

**VDXL**: Raw reading of the voltage applied to the Dynamixel bus. Scale factor: (3.3 / 1024) * (2.0/(2.0+10.0))

**VBAT_{0..5}**: Raw reading of the voltage between ground and cell {0..5}.

**VBAT_0**: Scale factor: (3.3 / 1024) * (3.3/(1.2+3.3))

**VBAT_1**: Scale factor: (3.3 / 1024) * (3.6/(6.2+3.6))

**VBAT_2**: Scale factor: (3.3 / 1024) * (2.2/(6.8+2.2))

**VBAT_3**: Scale factor: (3.3 / 1024) * (3.6/(16.0+3.6))

**VBAT_4**: Scale factor: (3.3 / 1024) * (6.2/(36.0+6.2))

**VBAT_5**: Scale factor: (3.3 / 1024) * (1.8/(13.0+1.8))


.. _Current Sensing:

Current Sensing
---------------

Current is sensed using a Hall effect sensor (ACS712ELCTR-30A-T to be exact).
It has to be scaled by the conversion factor from analog reading to voltage multiplied by the amperes per volt to get the actual current.
Furthermore, the reading is offset by 2.5V since the sensor can measure positive and negative currents.

**current**: Raw reading of the current sensor. Scale factor: (3.3 / 1024)) - 2.5) / -0.066
