=========================
Wolfgang Constant Voltage
=========================

Wolfang Constant voltage a currently nonfunctional attempt to provide a constant voltage to the motors of the robot.

We have observed that the motors behave very differently when powered at different voltage.
This causes stability issues in basically all motions of the robot.

To fix this problem the proposed solution is to use a 6-cell LiPo which provides 22.2V-25.2V.
This voltage is regulated to 16V using a DC-DC step down converter.

We have tested the `I6A24014A033V-001-R <https://product.tdk.com/de/search/power/switching-power/dc-dc-converter/info?part_no=i6A24014A033V-001-R>`_
but have found that the current surge at the beginning of motions caused by many motors trying to overcome the static friction triggers the overcurrent protection of the device.

We are currently investigating its successor, the `i7A4W033A033V-001-R <https://product.tdk.com/de/search/power/switching-power/dc-dc-converter/info?part_no=i7A4W033A033V-001-R>`_.

Schematics and board files will be published as soon as it is verified.
