=======
BitFoot
=======

The BitFoot is the foot pressure sensor by the Hamburg Bit-Bots.
It features a higher update rate and theoretically higher precision than the `Rhoban ForceFoot <https://www.github.com/Rhoban/ForceFoot>`_ on which it is based.

Firmware and schematics can be found in the `BitFoot git repository <https://www.github.com/bit-bots/bit_foot>`_.


Flashing the Generic STM32F103 Board (Blue Pill)
================================================

To flash the software to the Blue Pill over USB, a bootloader has to be flashed on the device.
For this, we need a FT232 breakout board similar to `this one <https://www.amazon.de/FT232RL-FTDI-USB-auf-TTL-Serienadapter-Arduino/dp/B00HSXDGOE>`_ or a different USB to serial converter and a few jumper wires.

Firstly download the `Arduino IDE <https://www.arduino.cc/en/Main/Software>`_, extract it and run the install.sh script.

Then download the `hardware library <https://github.com/rogerclarkmelbourne/Arduino_STM32>`_, extract it and place it in your hardware folder of the Arduino folder.
Per default, this folder should be located in your home directory.
If the hardware folder does not exist it, create it.
The path to the library should be:

:code:`~/Arduino/hardware/Arduino_STM32`

Download the correct bootloader binary (generic_boot20_pc13.bin) for the Board `here <https://github.com/rogerclarkmelbourne/STM32duino-bootloader/tree/master/binaries>`_.

Next we connect some wires:

* Set the header on the USB to Serial converter to 3.3V.
* Connect 3.3V of the USB Serial converter to 3.3V of the Blue Pill.
* Connect GND of the USB Serial converter to GND of the Blue Pill.
* Connect RX of the USB to Serial Converter to PA9 of the Blue Pill.
* Connect TX of the USB to Serial Converter to PA10 of the Blue Pill.
* Put the boot 0 pin on the STM32 board to 1 (top header when looking from the top and the USB port is left)

Connect the USB to serial converter to your computer using a Micro USB cable.


Run the install script in the ``Arduino_STM32`` folder

:code:`./Arduino_STM32/tools/linux64/install.sh`

Logout and back in again.
Then remove and replug the USB to serial device.

Locate the script for flashing the bootloader in the ``Arduino_STM32`` directory:

:code:`Arduino_STM32/tools/linux64/stm32flash`

Run the flash script:

:code:`./stm32flash -w <path to generic_boot20_pc13.bin> -v /dev/ttyUSB0 -b 115200`

When the bootloader is successfully installed, remove the USB to serial converter and its connections to the STM32 board.
Then place the boot 0 pin back to 0.

When connecting it to a computer using a Micro-USB cable, it should appear as a ``ttyACM0`` device in the ``/dev/`` folder.

Clone the `repository <https://www.github.com/bit-bots/bit_foot>`_ and open ``bit_foot/firmware/main/main.ino`` in the Arduino IDE.

Since the ``Arduino_STM32`` folder should already be in your Arduino directory, the board should already be installed.

You still need to install the ``Arduino SAM`` boards from the ``Board Manager`` to install the correct compiler.

Before compiling select:

* Board: ``Generic STM32F103C series``
* Variant: ``STM32F103C8 20k RAM 64k Flash`` (The other variants may also work)
* CPU Speed: ``72 Mhz``
* Upload Method: ``STM32duino Bootloader``
* Optimize: not clear yet... (use default for now)
* Select your port to be ``ttyACM0`` (or ``ttyACM1....N`` if 0 is not available)

Furthermore add the `ADS126X library <https://github.com/Molorius/ADS126X>`_ to the ArduinoIDE under Sketch.
Include library.
Add ``.zip`` library.

For the left foot, the ``DXL_ID`` should be set to ``102``, for the right foot, it should be ``101``.

Finaly compile your sketch.
When the compilation is finished, press the reset button on the board and the press the upload button in the Arduino IDE immediately.


Calibrating the Sensors
=======================

After the device has been connected to the DXL Bus, launch the hardware interface:

:code:`roslaunch bitbots_ros_control ros_control_pressure_only_standalone.launch`

Then run the calibration node:

:code:`rosrun bitbots_ros_control pressure_calibration.py`

The node will guide you through the process of calibrating the cleats.
