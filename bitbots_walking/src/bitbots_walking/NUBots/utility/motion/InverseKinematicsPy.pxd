from libcpp.vector cimport vector
from libcpp.pair cimport pair
from bitbots_common.pose.pypose cimport PyPose
from libcpp cimport bool

cdef extern from "InverseKinematicsHelper.hpp":
    cdef struct _Coordinates "Coordinates":
        float x
        float y
        float z
        float alpha
        float beta
        float gamma


    vector[float] berechneIK(_Coordinates Left, _Coordinates Right, _Coordinates Oberkoerper)
    _Coordinates calculateLegDirektKinematics(const vector[float]& positionen, bool Links)

cpdef berechneInverseKinematics(PyPose pose, list Left, list Right, list Oberkoerper)

cpdef berechneDirekteKinematic(PyPose pose)