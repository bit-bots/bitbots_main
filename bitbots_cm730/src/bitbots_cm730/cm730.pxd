from bitbots_common.utilCython.pydatavector cimport PyIntDataVector as IntDataVector
from bitbots_common.utilCython.pydatavector cimport PyDataVector as DataVector
from bitbots_common.utilCython.datavector cimport DataVector as CDataVector
from bitbots_common.utilCython.datavector cimport IntDataVector as CIntDataVector
from bitbots_common.utilCython.datavector cimport DataVector as CDataVector

from .lowlevel.controller.controller import BulkReadPacket
from libcpp cimport bool


cdef class CM730(object):
    cdef list read_packet_stub
    cdef BulkReadPacket read_packet2
    cdef list read_packet3_stub
    cdef last_io_success
    cdef int low_voltage_counter
    cdef bool dxl_power
    cdef int sensor_all_cid

    cdef init_read_packet(self)
    cpdef update_sensor_data(self)
    cpdef sensor_data_read(self)
    cpdef apply_goal_pose(self)
    cpdef switch_motor_power(self, state)
    cdef parse_sensor_data(self, object sensor_data, int cid_all_values)
    cdef set_motor_ram(self)

