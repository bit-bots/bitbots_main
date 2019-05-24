=======
BitFoot
=======

The BitFoot is the foot pressure sensor by the Hamburg Bit-Bots. It features a higher update rate
and theoretically higher precision than the Rhoban ForceFoot_ on which it is based.

Firmware and schematics can be found in the git repository_:


.. _ForceFoot: https://www.github.com/Rhoban/ForceFoot
.. _repository: https://www.github.com/bit-bots/bit_foot


Flashing the Generic STM32F103 Board (Blue Pill)
================================================

To flash the software to the device, a bootloader has to be flashed on the Blue Pill. For this we need a FT232 breakout board
similar to this_ one or a different USB to serial converter and a few jumper wires.

.. _this: https://www.amazon.de/FT232RL-FTDI-USB-auf-TTL-Serienadapter-Arduino/dp/B00HSXDGOE

Firstly download the Arduino IDE.

Then download the `hardware library`_ and place it in your hardware folder of the Arduino folder.

.. _hardware library: https://github.com/rogerclarkmelbourne/Arduino_STM32

Download the correct bootloader (generic_boot20_pc13.bin) for the Board here_.

.. _here: https://github.com/rogerclarkmelbourne/STM32duino-bootloader/tree/master/binaries

* Set the header on the USB to Serial converter to 3.3V.
* Connect 3.3V and GND of the USB Serial converter to the Blue Pill.
* Connect RX of the USB to Serial Converter to PA9 of the Blue Pill.
* Connect TX of the USB to Serial Converter to PA10 of the Blue Pill.
* Put the boot 0 pin to 1 (top jumper when looking from the top and the USB port is left)

Locate the script for flashing the bootloader in the Arduino_STM32 directory:


:code:`Arduino_STM32/tools/linux64/stm32flash`

Run the flash script:

:code:`stm32flash -w -b -v 115200 <path to binary>  /dev/ttyUSB0`

When the bootloader is successfully installed, place the boot pin back to 0.

When connecting it to a computer using a micro usb cable it should appear as a ttyACM0 device in the /dev/ folder.

Open the main sketch from the repository_ in the Arduino IDE.
Since the Arduino_STM32 folder should already be in your Arduino directory, the board should already be installed.

You still need to install the Arduino SAM boards from the Board Manager to install the correct compiler.

Before compiling select:

* Board: Generic STM32F103C series
* Variant: STM32F103C8 20k RAM. 64k Flash (The other variant may also work)
* CPU Speed: 72 Mhz
* Upload Method: STM32duino Bootloader
* Optimize: not clear yet...
* Select your port to be ttyACM0 (or ttyACM1....N is 0 is not available)

For the left foot, the DXL_ID should be set to 100, for the right foot, it should be 101.


Calibrating the Sensors
=======================
