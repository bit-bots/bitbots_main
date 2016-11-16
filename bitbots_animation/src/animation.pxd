#-*- coding:utf-8 -*-

from bitbots_common.pose.pypose cimport PyPose as Pose
from bitbots_ipc.ipc cimport AbstractIPC

"""
cdef class Keyframe:
    cdef public float duration
    cdef public float pause
    cdef public dict goals
    cdef public dict p
"""

cdef class Animation:

    cdef readonly object name

    cdef public list keyframes
    cdef public dict interpolators
    cdef public object default_interpolator

    cpdef get_interpolator(self, bytes name)
    cpdef list get_steps(self, bytes name)

cpdef parse(dict info)

cdef class Animator:

    cdef dict interpolators
    cdef readonly object name
    cdef readonly float time_min
    cdef readonly float time_max
    cdef readonly float duration


    cpdef get_pose(self, float t, Pose pose=?)

    cpdef play(self, AbstractIPC ipc, float stepsize=?, sleep=?, recordflag=?)
"""
cdef class Step:
    cdef public int off, hold
    cdef public float time
    cdef public float value
    cdef public float m
    cdef public int p


cdef class Interpolator:
    cdef readonly tuple steps
    cdef readonly float time_min, time_max

    cpdef prepare(self)
    cpdef tuple interpolate(self, float t)


cdef class LinearInterpolator(Interpolator):

    cpdef tuple interpolate(self, float t)

cdef float cubic_hermite_interpolate(Step a, Step b, float t)

cdef class CubicHermiteInterpolator(Interpolator):

    cpdef tuple interpolate(self, float t)

cdef class CatmullRomInterpolator(CubicHermiteInterpolator):
    cdef float m(self, int idx)
    cpdef prepare(self)

"""
