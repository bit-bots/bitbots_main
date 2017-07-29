# -*- encoding: utf8 -*-
from bitbots_common.pose.pose cimport Pose as CPose
from bitbots_common.pose.pypose cimport PyPose, wrap_pose
from libcpp cimport bool
from libcpp.string cimport string
from bitbots_common.eigen cimport Vector3f
from bitbots_common.eigen cimport Vector2d
from bitbots_common.eigen cimport Vector3d
from bitbots_common.eigen cimport Vector4d

cdef extern from "RhobanWalking/IKWalk.hpp":
    cdef struct _IKWalkParameters "Rhoban::IKWalkParameters":
        double distHipToKnee
        double distKneeToAnkle
        double distAnkleToGround
        double distFeetLateral
        double freq
        double enabledGain
        double supportPhaseRatio
        double footYOffset
        double stepGain
        double riseGain
        double turnGai;
        double lateralGain
        double trunkZOffset
        double swingGain
        double swingRollGain;
        double swingPhase;
        double stepUpVel;
        double stepDownVel;
        double riseUpVel;
        double riseDownVel;
        double swingPause;
        double swingVel;
        double trunkXOffset;
        double trunkYOffset;
        double trunkPitch;
        double trunkRoll;
        double extraLeftX;
        double extraLeftY;
        double extraLeftZ;
        double extraRightX;
        double extraRightY;
        double extraRightZ;
        double extraLeftYaw;
        double extraLeftPitch;
        double extraLeftRoll;
        double extraRightYaw;
        double extraRightPitch;
        double extraRightRoll;

cdef extern from "RhobanWalking/walking.hpp":

    cdef enum FootPhase "RhobanWalk::FootPhase":
        left, right, stopped, both

    cdef cppclass _RhobanWalk "RhobanWalk":
        CPose& get_pose()
        FootPhase update()
        void start()
        void stop()
        bool is_active()
        void set_velocity(float, float, float)
        void set_gyro(float, float, float)
        void set_frequenzy(float)
        _IKWalkParameters& getParams()



cdef class ZMPWalkingEngine:
    cdef _RhobanWalk* thisptr
    cdef _IKWalkParameters* parameters
    cpdef start(self)
    cpdef stop(self)
    cpdef process(self)
    cpdef set_velocity(self, float x, float y, float z)
    cpdef setParams(self, object config)
    cpdef set_gyro(self, float, float, float)
    cpdef set_active(self, bool active)
    cpdef stance_reset(self)
    cpdef create_walkready_pose(self, dict config=?, duration=?)
