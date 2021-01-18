======================
Bitbots IMU DXL Module
======================

`Github repository <https://github.com/bit-bots/bitbots_imu_dxl>`_


todo picture



Features
========

* 12 Bit 3 axis gyroscope and 3 axis accelerometer (MPU6500 sensor)
* 1000 Hz update rate
* configurable range (250-2000 deg/sec and 2-16g)
* configurable onboard complementary `filter <https://www.mdpi.com/1424-8220/15/8/19302>`_
* gyroscope and accelerometer calibration procedure
* direct reading of orientation as quaternion
* connection via the Dynamixel bus
* WS2812b RGB LEDs
* 3 Buttons


Firmware
========



ROS Control
===========






Register Table
==============

+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| Adress | Length | Name                                                | Access | Default | Type    | Persistent |
+========+========+=====================================================+========+=========+=========+============+
| 7      | 1      | :ref:`id<DXL>`                                      | rw     | 241     | int8    | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 8      | 1      | :ref:`baud<DXL>`                                    | rw     | 4       | int8    | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 10     | 4      | :ref:`led0<LEDs>`                                   | rw     | 0       | int8[4] | no         |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 14     | 4      | :ref:`led1<LEDs>`                                   | rw     | 0       | int8[4] | no         |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 18     | 4      | :ref:`led2<LEDs>`                                   | rw     | 0       | int8[4] | no         |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 36     | 4      | :ref:`gyro_x<IMU>`                                  | r      |         | float32 |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 40     | 4      | :ref:`gyro_y<IMU>`                                  | r      |         | float32 |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 44     | 4      | :ref:`gyro_z<IMU>`                                  | r      |         | float32 |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 48     | 4      | :ref:`accel_x<IMU>`                                 | r      |         | float32 |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 52     | 4      | :ref:`accel_y<IMU>`                                 | r      |         | float32 |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 56     | 4      | :ref:`accel_z<IMU>`                                 | r      |         | float32 |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 60     | 4      | :ref:`quaternion_x<IMU>`                            | r      |         | float32 |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 64     | 4      | :ref:`quaternion_y<IMU>`                            | r      |         | float32 |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 68     | 4      | :ref:`quaternion_z<IMU>`                            | r      |         | float32 |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 72     | 4      | :ref:`quaternion_w<IMU>`                            | r      |         | float32 |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 76     | 1      | :ref:`button0<Buttons>`                             | r      |         | int8    |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 77     | 1      | :ref:`button1<Buttons>`                             | r      |         | int8    |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 78     | 1      | :ref:`button2<Buttons>`                             | r      |         | int8    |            |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 102    | 1      | :ref:`gyro_range<Ranges>`                           | rw     | 3       | int8    | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 103    | 1      | :ref:`accel_range<Ranges>`                          | rw     | 3       | int8    | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 104    | 1      | :ref:`calibrate_gyro<IMU calibration>`              | rw     | 0       | int8    | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 105    | 1      | :ref:`reset_gyro_calibration<IMU calibration>`      | rw     | 0       | int8    | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 106    | 1      | :ref:`calibrate_accel<IMU calibration>`             | rw     | 0       | int8    | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 107    | 1      | :ref:`reset_accel_calibration<IMU calibration>`     | rw     | 0       | int8    | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 108    | 1      | :ref:`do_adaptive_gain<Complementary Filter>`       | rw     | 0       | int8    | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 109    | 1      | :ref:`do_bias_estimation<Complementary Filter>`     | rw     | 0       | int8    | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 110    | 4      | :ref:`accel_gain<Complementary Filter>`             | rw     | 0.04    | float32 | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 114    | 4      | :ref:`bias_alpha<Complementary Filter>`             | rw     | 0.01    | float32 | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 118    | 4      | :ref:`accel_calibration_threshold<IMU calibration>` | rw     | 7.5     | float32 | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 122    | 4      | :ref:`accel_bias_x<IMU calibration>`                | rw     | 0.0     | float32 | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 126    | 4      | :ref:`accel_bias_y<IMU calibration>`                | rw     | 0.0     | float32 | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 130    | 4      | :ref:`accel_bias_z<IMU calibration>`                | rw     | 0.0     | float32 | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 134    | 4      | :ref:`accel_scale_x<IMU calibration>`               | rw     | 1.0     | float32 | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 138    | 4      | :ref:`accel_scale_y<IMU calibration>`               | rw     | 1.0     | float32 | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+
| 142    | 4      | :ref:`accel_scale_z<IMU calibration>`               | rw     | 1.0     | float32 | yes        |
+--------+--------+-----------------------------------------------------+--------+---------+---------+------------+


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
| 1     | 57,600  | ?      |
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



.. _LEDs:

LEDs
----

**led{0,1,2}**: byte order: RGB


.. _IMU:

IMU
---

**gyro_{x,y,z}**: Current measurement of the gyroscope in the respective axis in rad/s

**accel_{x,y,z}**: Current measurement of the accelerometer in the respective axis in m/s^2

**quaternion_{x,y,z,w}**: Quaternion giving the orientation of the imu in respect to to ground.


.. _Buttons:

Buttons
-------

**button{0,1,2}**: Reading from the buttons, 1 when pressed.


.. _Ranges:

Ranges
------

**gyro_range**: Can be a value between 0 and 3

+-------+-------------+
| value | range       |
+=======+=============+
| 0     | ±250 deg/s  |
+-------+-------------+
| 1     | ±500 deg/s  |
+-------+-------------+
| 2     | ±1000 deg/s |
+-------+-------------+
| 3     | ±2000 deg/s |
+-------+-------------+

**accel_range**: Can be a value between 0 and 3

+-------+--------+
| value | range  |
+=======+========+
| 0     | ±2 g   |
+-------+--------+
| 1     | ±4 g   |
+-------+--------+
| 2     | ±8 g   |
+-------+--------+
| 3     | ±16 g  |
+-------+--------+


.. _IMU calibration:

IMU calibration
---------------

**calibrate_gyro**: Setting this value to 1 causes the gyroscope to be calibrated, the module is unresponsive for around 2 seconds
This procedure should be performed when the IMU is not moving.
It should be done relatively frequently.
The gyro calibration is not persistent.

**reset_gyro_calibration**: Resets the gyro calibration. Useful if the gyro was accidentally calibrated while moving.

**calibrate_accel**: Starts the :ref:`accelerometer calibration routine<Accelerometer Calibration>`.

**reset_accel_calibration**: Resets the :ref:`accelerometer calibration<Accelerometer Calibration>`.
Be careful as it can be tedious to perform the calibration routine.

**accel_calibration_threshold**: The threshold used for accelerometer :ref:`accelerometer calibration<Accelerometer Calibration>`.

**accel_bias_{x,y,z}**: The bias (i. e. the offset from 0) calculated in the calibration routine.

**accel_scale_{x,y,z}**: The scale factor calculated in the calibration routine. Should be relatively close to 1.0 after calibration.


.. _Complementary Filter:

Complementary Filter
--------------------

**do_adaptive_gain**: If 1, the gain is adapted to be weighted more if the IMU is in a steady state.

**do_bias_estimation**: If 1, the bias of the gyroscope is estimated when the IMU is in a steady state.

**accel_gain**: How much the orientation is influenced by the accelerometer.

**bias_alpha**: In the bias estimation, how strongly the biases are adjusted if **do_bias_estimation** is 1 and the IMU is in a steady state.


.. _Accelerometer Calibration:

Accelerometer Calibration
=========================

todo