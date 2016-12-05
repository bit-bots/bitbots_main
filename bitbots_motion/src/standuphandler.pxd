# -*- coding: utf8 -*-
from libcpp cimport bool
from bitbots_common.utilCython.pydatavector cimport PyDataVector as DataVector
from bitbots_common.utilCython.pydatavector cimport PyIntDataVector as IntDataVector



cdef class StandupHandler(object):
    cdef bool falling_activated
    cdef float falling_ground_coefficient
    cdef object falling_motor_degrees_front, falling_motor_degrees_back, falling_motor_degrees_left, falling_motor_degrees_right
    cdef float falling_threshold_front, falling_threshold_back, falling_threshold_left, falling_threshold_right

    cdef update_sensor_data(self, raw_gyro)
    cdef check_falling(self, not_much_smoothed_gyro)
    cdef check_falling_front_back(self, not_much_smoothed_gyro)
    cdef check_falling_sideways(self, not_much_smoothed_gyro)
    cdef check_fallen(self, raw_gyro, smooth_gyro, robo_angle)
    cdef set_falling_pose(self, object falling_motor_degrees, object goal_pose)