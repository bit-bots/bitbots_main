=============
Wolfgang CORE
=============

`Github repository <https://github.com/bit-bots/wolfgang_core>`_

.. image:: core.png
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

.. image:: core_bodge.png
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
