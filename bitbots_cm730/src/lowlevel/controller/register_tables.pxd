from bitbots_motion.lowlevel.controller.converter cimport Converter

from libc.stdint cimport uint8_t
ctypedef uint8_t ubyte


cdef class MX28RegisterTable:
    cdef readonly Register model, version, cid, baudrate, return_delay_time
    cdef readonly Register cw_angle_limit, ccw_angle_limit, system_data
    cdef readonly Register limit_temperatur, limit_voltage_low
    cdef readonly Register limit_voltage_high, max_torque, return_level
    cdef readonly Register alarm_led, alarm_shutdown, operating_mode
    cdef readonly Register low_calibration, high_calibration, torque_enable
    cdef readonly Register led, d, i, p, reserved, goal_position, moving_speed
    cdef readonly Register torque_limit, present_position, present_speed
    cdef readonly Register present_load, present_voltage, present_temperature
    cdef readonly Register registered_instruction, pause_time, moving, lock
    cdef readonly Register puch, reserved4, reserved5, pot, pwm_out, p_error
    cdef readonly Register i_error, d_error, p_error_out, i_error_out
    cdef readonly Register d_error_out


    cdef Register get_register_by_name(self, object name)


cdef class CM730RegisterTable:
    cdef readonly Register version, cid, baudrate
    cdef readonly Register dxl_power, led_panel, led_head, led_eye
    cdef readonly Register button, padding31_37, gyro, accel, voltage


cdef class Register:
    cdef int start
    cdef Converter conv

    cdef encode(self, object value, ubyte *result)
    cdef decode(self, ubyte *data)
    cdef int length(self)
    cdef int get_start(self)
