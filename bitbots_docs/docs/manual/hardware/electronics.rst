===========
Electronics
===========

Powerboard
==========

The Powerboard allows to connect either an external power supply or a battery. It has a switch to turn on and off the power to the servos.

* Schematic :download:`pdf <electronics/wolfgang_power.pdf>`
* PCB combined :download:`pdf <electronics/wolfgang_power_pcb.pdf>`
* PCB Top :download:`pdf <electronics/wolfgang_power_pcb_top.pdf>`
* PCB Bottom :download:`pdf <electronics/wolfgang_power_pcb_bottom.pdf>`
* PCB Silk :download:`pdf <electronics/wolfgang_power_pcb_silk.pdf>`

The footprint of the mosfet is currently wrong, but since the Wolfgang Core board integrates the functionality of this board, it will not be changed.


DXLBoard
========

The DXLBoard is developed by Rhoban from Bordeaux. It communicates with the Servo bus over RS485 or TTL and the NUC via USB.

Documentation about the Hardware is available here_. 
We use a custom firmware to support Dynamixel Protocol 2.0 which can be found on our fork_.

.. _here: https://github.com/Rhoban/DXLBoard
.. _fork: https://github.com/bit-bots/DXLBoard/tree/protocol2_single_bus


Wolfgang Core
=============

Wolfgang Core (COntrolling and Regulating Electronics) is currently in the testing phase.

* 4 buses with up to 10 MBaud (highest for Dynamixel MX and X servos is 4 MBaud)
* power switching with switch and software
* voltage regulation (9V 1A, 5V 5A)
* voltage sensing
* current sensing
* 3 RGB status leds


Bit Foot
========

The Bit Foot is a board to read 4 load cells on the foot of the robot and communicate over the dynamixel bus.

* Schematics :download:`pdf <electronics/bitfoot.pdf>`
* PCB :download:`pdf <electronics/bitfoot_pcb.pdf>`
