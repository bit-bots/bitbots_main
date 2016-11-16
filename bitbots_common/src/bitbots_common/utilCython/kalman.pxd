cimport numpy as np
from bitbots_common.utilCython.datavector cimport IntDataVector, DataVector
from bitbots_common.utilCython.pydatavector cimport PyIntDataVector, PyDataVector

cdef extern from "KalmanFilter.hpp" namespace "Util":
    cppclass SimpleKalman:
        SimpleKalman()
        SimpleKalman(double, double, double)

        double getAngle(double, double)
        double getAngle(double, double, double)
        double getRate()
        void setAngle(double)

cdef class Kalman(object):
    cdef SimpleKalman* kalman

    cdef float get_angle(self, int newRate, float dt)
    cdef float get_angle2(self, double angle, int newRate, float dt)
    cdef float get_angle3(self, double angle_in, float newRate, float dt)
    cdef int get_rate(self)
    cdef reset(self)

cdef class TripleKalman(object):
    cdef Kalman kalman_x
    cdef Kalman kalman_y
    cdef Kalman kalman_z

    cdef DataVector get_angles_pv(self, PyIntDataVector newRates, float dt)
    cdef DataVector get_angles_pvv(self, DataVector& angles_in, PyIntDataVector newRates, float dt)
    cdef DataVector get_angles_vv(self, DataVector& angles_in, IntDataVector newRates, float dt)
    cdef DataVector get_angles_vfv(self, DataVector& angles_in, DataVector newRates, float dt)
    cdef DataVector get_angles_v(self, IntDataVector newRates, float dt)
    cdef PyIntDataVector get_rates_v(self)
    cdef float get_angle_x(self, int newRate_x, float dt)
    cdef float get_angle_y(self, int newRate_y, float dt)
    cdef float get_angle_z(self, int newRate_z, float dt)
    cdef int get_rate_x(self)
    cdef int get_rate_y(self)
    cdef int get_rate_z(self)
    cdef reset(self)

#IF EXTEND_KLAMAN_API:
    cdef tuple get_angles_t(self, int newRate_x, int newRate_y, int newRate_z, float dt)
    cdef np.ndarray get_angles_a(self, np.ndarray newRates, float dt)
    cdef np.ndarray get_angles(self, int newRate_x, int newRate_y, int newRate_z, float dt)
#ENDIF
