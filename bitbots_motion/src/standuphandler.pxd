# -*- coding: utf8 -*-
from libcpp cimport bool
from bitbots_common.utilCython.pydatavector cimport PyDataVector as DataVector
from bitbots_common.utilCython.pydatavector cimport PyIntDataVector as IntDataVector
cdef public enum:
    FALLEN_BOTTOM_UP = 1
    FALLEN_FRONT_UP
    FALLEN_BEND_FORWARD
    FALLEN_SQUATTED
    FALLEN_UPRIGHT



cdef class StandupHandler(object):
    cdef bool falling_activated
    cdef float falling_ground_coefficient
    cdef object falling_motor_degrees_front, falling_motor_degrees_back, falling_motor_degrees_left, falling_motor_degrees_right
    cdef float falling_threshold_front, falling_threshold_back, falling_threshold_left, falling_threshold_right
    cdef int fallState
    cdef dict config
    cdef DataVector not_much_smoothed_gyro
    cdef IntDataVector raw_gyro

    cdef update_sensor_data(self, raw_gyro)
    cdef get_up(self, set_state)
    cdef load_falling_data(self, config)
    cdef check_fallen(self, object goal_pose , object set_state, state, smooth_gyro, robo_angle, smooth_accel)
    cdef check_fallen_forwardAndBackward(self, set_state, state)
    cdef check_fallen_sideways(self, set_state , state)
    cdef set_falling_pose(self, object falling_motor_degrees, object goal_pose)
    cdef info(self, text)

    #STATE_CONTROLABLE, STATE_FALLING, STATE_FALLEN, STATE_GETTING_UP, \
    #STATE_ANIMATION_RUNNING, STATE_BUSY, STATE_STARTUP, STATE_PENALTY, STATE_PENALTY_ANIMANTION, STATE_RECORD, \
    #STATE_SOFT_OFFSTATE_WALKING