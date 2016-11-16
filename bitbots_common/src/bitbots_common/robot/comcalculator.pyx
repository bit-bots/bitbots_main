from cython.operator cimport dereference as deref
import time

from bitbots.robot.pose cimport Pose
from bitbots.robot.pypose cimport PyPose

cdef import from "com_calculator.hpp":
    cppclass _Calculator "Robot::Kinematics::Center_Of_Mass_Calculator":
        _Calculator()
        void update(Pose& pose)
        void print_chains(Pose& pose)
        void lost_leg()

cdef class Calculator(object):
    cdef _Calculator* calculator

    def __cinit__(self):
        self.calculator = new _Calculator()

    def __dealloc__(self):
        del self.calculator

    def update(self, PyPose pose):
        #self.calculator.update(deref(pose.pose))
        self.calculator.print_chains(deref(pose.pose))

    def lost_leg(self):
        self.calculator.lost_leg()
