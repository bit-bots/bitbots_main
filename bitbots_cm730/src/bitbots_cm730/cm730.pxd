#todo vom alten basemotion und motion server holen

from bitbots_common.eigen cimport *

from bitbots_motion.lowlevel.controller.controller cimport Controller, BulkReadPacket
from bitbots_common.utilCython.datavector cimport IntDataVector as CIntDataVector
from bitbots_common.utilCython.datavector cimport DataVector as CDataVector
from bitbots_common.utilCython.pydatavector cimport PyIntDataVector as IntDataVector
from bitbots_common.utilCython.pydatavector cimport PyDataVector as DataVector
from .robot.kinematics cimport Robot

cdef class cm730(object):

    cdef extern from "cmath" namespace "std":
        double asin(double)
        double acos(double)

    cdef extern from "walking/zmp_math_basics.h":
        double rad_to_degree, degree_to_rad

    cdef inline double calc_sin_angle(const Vector3d& fst, const Vector3d& sec):
        if(fst.norm() == 0 or sec.norm() == 0):
            return 0 #TODO Rückgabewert sinvoll?
        return asin(fst.dot(sec) / (fst.norm() * sec.norm())) * rad_to_degree

    cdef inline double calc_cos_angle(const Vector3d& fst, const Vector3d& sec):
        return acos(fst.dot(sec) / (fst.norm() * sec.norm())) * rad_to_degree

    cdef inline CDataVector calculate_robot_angles(const CIntDataVector& rawData):
        cdef Vector3d raw = Vector3d(rawData.get_x(), rawData.get_y(), rawData.get_z())
        cdef double roll_angle, pitch_angle, yaw_angle

        pitch_angle = calc_sin_angle(raw, unitY3d())
        if(raw.z() < 0 and raw.y() < 0):
            pitch_angle = - pitch_angle - 180
        elif(raw.z() < 0 and raw.y() > 0):
            pitch_angle = 180 - pitch_angle

        roll_angle = calc_sin_angle(raw, unitX3d())

        #TODO mir ist noch keiner schlaue Formel für diesen Wingkel eingefallen
        yaw_angle = 0

        #print "pitch %f, roll %f" % (pitch_angle, roll_angle)

        return CDataVector(-roll_angle, -pitch_angle, yaw_angle)