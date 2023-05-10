=========================
Wolfgang Constant Voltage
=========================

Wolfang Constant Voltage regulates the voltage of the battery to 15.4V (this may be configured however).

We have observed that the motors behave very differently when supplied with different voltage.
This causes stability issues in basically all motions of the robot depending on the battery charging state.

Therefore, we decided to solve this problem using a 6-cell LiPo battery which provides 22.2V-25.2V and regulate the voltage down to be usable by the motors.
The step-down converter is a `TDK-Lambda i7A4W033A033V-0C1-R <https://product.tdk.com/en/search/power/switching-power/dc-dc-converter/info?part_no=i7A4W033A033V-0C1-R>`_.
This module can regulate up to 33A continously and 45A peak.
For our application, this is sufficient.
Several versions of the Board are available with differen heatsinks.
We found that the `i7A4W033A033V-0C1-R <https://product.tdk.com/en/search/power/switching-power/dc-dc-converter/info?part_no=i7A4W033A033V-0C1-R>`_ does not get too warm during operation.
The version with a larger heatsink may be more efficient at the cost of weight.

Previously, we have tested the `I6A24014A033V-001-R <https://product.tdk.com/de/search/power/switching-power/dc-dc-converter/info?part_no=i6A24014A033V-001-R>`_
but have found that the current surge at the beginning of some motions caused by many motors trying to overcome the static friction,
and motions requiring many motors (e. g. standing up)  triggers the overcurrent protection of the device.

Schematics, board drawings and gerber files for production are available in the `GitHub repository <https://github.com/bit-bots/wolfgang_constant_voltage>`_.

R3 configures the output voltage of the converter. Several elatively common values as well as the calculation for a desired voltage are given in the schematic.

The SENSE header can be connected to an external ADC to measure the output voltage of the converter. The range is defined by the voltage divider R1 and R2.

The input and output capacitors are chosen according to the datasheet of the converter.

