# -*- coding: utf8 -*-

from bitbots_common.utilCython.pydatavector cimport PyIntDataVector as IntDataVector
from bitbots_common.utilCython.pydatavector cimport PyDataVector as DataVector
from bitbots_common.utilCython.datavector cimport DataVector as CDataVector
from bitbots_common.utilCython.datavector cimport IntDataVector as CIntDataVector
from bitbots_common.utilCython.datavector cimport DataVector as CDataVector
from bitbots_common.pose.pypose cimport PyPose as Pose


from bitbots_cm730.lowlevel.controller.controller cimport BulkReadPacket, Controller
from libcpp cimport bool


cdef class CM730(object):
    cdef bool using_cm_730
    cdef Controller ctrl
    cdef list read_packet_stub
    cdef BulkReadPacket read_packet2
    cdef list read_packet3_stub
    cdef last_io_success
    cdef int low_voltage_counter
    cdef dict last_overload
    cdef dict overload_count
    cdef int button1
    cdef int button2
    cdef Pose robo_pose
    cdef bool dxl_power
    cdef int sensor_all_cid
    cdef IntDataVector raw_gyro
    cdef IntDataVector robo_accel
    cdef list motors
    cdef dict motor_ram_config
    cdef dict motor_rom_config
    cdef list joint_offsets
    cdef list joints
    cdef bool eye_param

    cpdef init_read_packet(self)
    cpdef sensor_data_read(self)
    cpdef apply_goal_pose(self, object goal_pose)
    cpdef switch_motor_power(self, bool state)
    cpdef parse_sensor_data(self, object sensor_data, int cid_all_values)
    cpdef set_motor_ram(self)

