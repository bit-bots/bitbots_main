Bitbots Lowlevel
================

What do the low level packages do?
---------------------------------

The low level packages are responsible for the motions of the robot.
For that, different hardware and software components are used; they are described in this article.
Additionally, common problems and troubleshooting strategies are explained.

The behavior of the low level packages is dominated by the control loop, i.e. a cycle that alternately reads and writes motor positions.
In parallel to this cycle, the new positions are calculated based on the observed values.

To react to problems as fast as possible, e.g. to overload errors in the motors, a faster control loop is desirable.
Formerly, our loop ran with a frequency of 100Hz, therefore the reaction time to errors it at least 20ms because the error has to be read, processed and written, which takes at least two cycles.

There are essentially three possibilities to accelerate the control loop:

1. Send the bits faster over the bus, but this is limited by the baud rate of up to four megabaud (Robotis claims to achieve 4.5 MBaud, but this could not be reproduced)
2. Compress the data, e.g. by using special commands to read multiple motors at once (sync read and sync write)
3. Use more buses, in our case, one bus per limb would make sense

How is the hardware control structured?
---------------------------------------

Motors and Buses
~~~~~~~~~~~~~~~~

The lowest end of the hardware control are the motors.
We use Dynamixel motors MX-106 and MX-64 by Robotis.
The motors are connected by the motorbus, we are using RS-485 or TTL.

The cables for RS-485 consist of four wires, two of which are ground and VCC (14.8 to 16.8 V, depending on the current battery voltage), one for data (5V) and one for the inverted data.
When crimping new cables or when connecting a logic analyzer, it is important to not interchange the data and VCC cables because the motors are damaged when there are more than 5V on the data bus.

The cables for TTL only have three wires: Ground, VCC and data.
For the correct crimping, the same as for the RS-485 cables applies.

The communication with the motors happens via the Dynamixel Bus Protocol.
Details are documented in their protocol specification.
We are using protocol `version 2<http://emanual.robotis.com/docs/en/dxl/protocol2>`_, but protocol `version 1<http://emanual.robotis.com/docs/en/dxl/protocol1/>`_ can be used as a reference as well.

A message of the protocol essentially consists of a header, the goal motor id, an instruction, a parameter list and a check sum.
The most important instructions are ping, read, write and status as well as sync read and sync write to read/write multiple motors at once.

Every motor has a fixed id by which it can be addressed.
The id of a new / reset motor is 1 but can be changed using a write instruction to a special register.

Two motors with the same id may never be connected to the same bus because of the resulting communication problems.

On a sync read, the motors answer in the order by which they are addressed in the sync read.
When one of the motors does not answer, the following motors will also not answer because they wait for the previous motor.

DXL Board
~~~~~~~~~

The DXL Board is connected to the NUC via USB and to the motors via the motor bus.
It takes care of the communication between the NUC and the motors by reading message from the bus and redirecting them to the NUC and vice versa.
The IMU is also located on the bus and its values are treated the same as the motor values.

Kernel
~~~~~~

From here, all levels of the hardware control are on the NUC.

In the context of hardware control, the Linux kernel provides the connected DXL Board to the software using it.
It is important to set the kernel latency from 16ms to 0, e.g. by typing

.. code:: bash

   sudo sh -c "echo 0 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"

This is only necessary for "real" serial devices.
The DXL Board is currently registered as an ACM, therefore this is not required.
For the new QUADDXL this will be extremely important, though.

Dynamixel SDK
~~~~~~~~~~~~~

The Dynamixel SDK implements the Dynamixel protocol.
It provides methods to send instructions and to read status packets in different programming languages.
We use a fork of Robotis' Dynamixel SDK because Robotis did not implement the sync read on multiple registers.

Dynamixel Workbench
~~~~~~~~~~~~~~~~~~~

The Dynamixel Workbench provides higher level functions than the Dynamixel SDK.
For example, the motor positions in the SDK are given as values between 0 and 4096 (2 Byte) which is converted to radians by the Dynamixel Workbench.
Thereby, the Workbench eases the work with the motors on a more abstract level.

ROS Control Framework
~~~~~~~~~~~~~~~~~~~~~

The ROS Control Framework is a part of ROS that is responsible for the motor and sensor control.
There are controllers for ROS Control that provide the interface between ROS and low level software parts.
These controllers are hardware agnostic because they are using interfaces to abstract from the hardware (e.g. motors).
To control the motors, the Dynamixel Controller is used, which itself uses the Dynamixel Hardware Interface.

ROS messages
~~~~~~~~~~~~

After all these steps comes finally the ROS message level.
This 
