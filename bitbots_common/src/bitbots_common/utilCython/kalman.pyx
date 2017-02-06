from cython.operator cimport dereference as deref

from bitbots_common.utilCython.pydatavector cimport wrap_int_data_vector
from bitbots_common.utilCython.pydatavector cimport PyIntDataVector, PyDataVector

cdef float raw_to_dps_factor = 3.125

cdef class Kalman(object):

    def __cinit__(self):
        self.kalman = new SimpleKalman()

    def __dealloc__(self):
        del self.kalman

    cdef float get_angle(self, int newRate, float dt):
        """
            @param newRate: raw gyro value in range of -512 to 511. Representing degree per second
            @param dt: Time between updates in seconds
            @return: returns the current angle in degree
        """
        cdef float angle = self.kalman.getAngle(newRate * raw_to_dps_factor, dt)
        return angle

    cdef float get_angle2(self, double angle_in, int newRate, float dt):
        """
            @param newRate: raw gyro value in range of -512 to 511. Representing degree per second
            @param angle: estimated robot angle
            @param dt: Time between updates in seconds
            @return: returns the current angle in degree
        """
        cdef float angle = self.kalman.getAngle(angle_in, newRate * raw_to_dps_factor, dt)
        return angle

    cdef float get_angle3(self, double angle_in, float newRate, float dt):
        """
            @param newRate: Representing degree per second
            @param angle: estimated robot angle
            @param dt: Time between updates in seconds
            @return: returns the current angle in degree
        """
        cdef float angle = self.kalman.getAngle(angle_in, newRate, dt)
        return angle

    cdef int get_rate(self):
        """
            @return returns a emulated filtered raw value
        """
        return int(self.kalman.getRate() / raw_to_dps_factor)

    cdef reset(self):
        self.kalman.setAngle(0)

cdef class TripleKalman(object):

    def __cinit__(self):
        self.kalman_x = Kalman()
        self.kalman_y = Kalman()
        self.kalman_z = Kalman()

    cdef DataVector get_angles_pv(self, PyIntDataVector newRates, float dt):
        return self.get_angles_v(deref(newRates.get_data_vector()), dt)

    cdef DataVector get_angles_v(self, IntDataVector newRates, float dt):
        cdef DataVector angles = DataVector()
        angles.set_x(self.kalman_x.get_angle(newRates.get_x(), dt))
        angles.set_y(self.kalman_y.get_angle(newRates.get_y(), dt))
        angles.set_z(self.kalman_z.get_angle(newRates.get_z(), dt))
        return angles

    cdef DataVector get_angles_pvv(self, DataVector& angles_in, PyIntDataVector newRates, float dt):
        return self.get_angles_vv(angles_in, deref(newRates.get_data_vector()), dt)

    cpdef PyDataVector get_angles_pvv_py(self, PyDataVector angles_in, PyIntDataVector newRates, float dt):
        cdef DataVector vec =  self.get_angles_vv(deref(angles_in.get_data_vector()), deref(newRates.get_data_vector()), dt)
        cpdef PyDataVector py_vec = PyDataVector(vec.get_x(), vec.get_y(), vec.get_z())
        return  py_vec

    cdef DataVector get_angles_vv(self, DataVector& angles_in, IntDataVector newRates, float dt):
        cdef DataVector angles = DataVector()
        angles.set_x(self.kalman_x.get_angle2(angles_in.get_x(), newRates.get_x(), dt))
        angles.set_y(self.kalman_y.get_angle2(angles_in.get_y(), newRates.get_y(), dt))
        angles.set_z(self.kalman_z.get_angle(newRates.get_z(), dt))
        return angles

    cdef DataVector get_angles_vfv(self, DataVector& angles_in, DataVector newRates, float dt):
        cdef DataVector angles = DataVector()
        angles.set_x(self.kalman_x.get_angle3(angles_in.get_x(), newRates.get_x(), dt))
        angles.set_y(self.kalman_y.get_angle3(angles_in.get_y(), newRates.get_y(), dt))
        angles.set_z(self.kalman_z.get_angle3(0.0, newRates.get_z(), dt))
        return angles

    cdef PyIntDataVector get_rates_v(self):
        cdef IntDataVector rates = IntDataVector()
        rates.set_x(self.kalman_x.get_rate())
        rates.set_y(self.kalman_y.get_rate())
        rates.set_z(self.kalman_z.get_rate())
        return wrap_int_data_vector(rates)

    cdef float get_angle_x(self, int newRate_x, float dt):
        return self.kalman_x.getAngle(newRate_x, dt)

    cdef float get_angle_y(self, int newRate_y, float dt):
        return self.kalman_y.getAngle(newRate_y, dt)

    cdef float get_angle_z(self, int newRate_z, float dt):
        return self.kalman_z.getAngle(newRate_z, dt)

    cdef int get_rate_x(self):
        return self.kalman_x.getRate()

    cdef int get_rate_y(self):
        return self.kalman_y.getRate()

    cdef int get_rate_z(self):
        return self.kalman_z.getRate()

    cdef reset(self):
        self.kalman_x.reset()
        self.kalman_y.reset()
        self.kalman_z.reset()

#IF EXTEND_KLAMAN_API:

    cdef tuple get_angles_t(self, int newRate_x, int newRate_y, int newRate_z, float dt):
        cdef tuple angles = (
            self.kalman_x.get_angle(newRate_x, dt),
            self.kalman_y.get_angle(newRate_y, dt),
            self.kalman_z.get_angle(newRate_z, dt)
        )
        return angles

    cdef np.ndarray get_angles_a(self, np.ndarray newRates, float dt):
        return self.get_angles(newRates[0], newRates[1], newRates[2], dt)

    cdef np.ndarray get_angles(self, int newRate_x, int newRate_y, int newRate_z, float dt):
        cdef np.ndarray angles = np.ndarray(
            self.kalman_x.get_angle(newRate_x, dt),
            self.kalman_y.get_angle(newRate_y, dt),
            self.kalman_z.get_angle(newRate_z, dt)
        )
        return angles
 #ENDIF
