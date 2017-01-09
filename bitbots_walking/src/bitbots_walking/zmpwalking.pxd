# -*- encoding: utf8 -*-
from bitbots_common.pose.pose cimport Pose as CPose
from libcpp cimport bool
from libcpp.string cimport string
from bitbots_common.eigen cimport Vector2d
from bitbots_common.eigen cimport Vector3d
from bitbots_common.eigen cimport Vector4d

cdef extern from "walking/zmp_walk_wrapper.hpp":

    cdef enum FootPhase "ZMPWalk::FootPhase":
        left, right, stopped, both

    cdef cppclass _ZMPWalk "ZMPWalking::ZMPWalkWrapper":
        CPose& get_pose()
        double get_l_shoulder_pitch_offset()
        double get_r_shoulder_pitch_offset()
        double get_l_shoulder_roll_offset()
        double get_r_shoulder_roll_offset()
        double get_hip_pitch_offset()
        string get_robottype()
        FootPhase update()
        void set_l_shoulder_pitch_offset(double)
        void set_r_shoulder_pitch_offset(double)
        void set_robottype(string)
        void set_l_shoulder_roll_offset(double)
        void set_r_shoulder_roll_offset(double)
        void set_hip_pitch_offset(double)
        void start()
        void start_motion(string name)
        void stance_reset()
        void stop()
        bool is_active()
        void set_active(bool active)
        void set_velocity(float, float, float)
        const Vector3d& get_velocity()
        const Vector3d& get_uLeft()
        const Vector3d& get_uRight()
        void set_gyro_data(const Vector3d& gyro)
        void set_hip_pitch(double hip_pitch)
        double get_hip_pitch()
        void set_long_legs(double thight, double tibia, double hip_y_offset, double hip_z_offset, double foot_height)
        void set_belly_roll_pid(float p, float i, float d)

    _ZMPWalk* getInstance(const _ZMPParameter& parameter)

cdef extern from "walking/zmp_walk_parameter.hpp":
    cdef cppclass _ZMPParameter "ZMPWalking::ZMPParameter":
        _ZMPParameter()
        _ZMPParameter(_ZMPParameter)
        double bodyHeight
        double stepHeight
        double tStep
        double tZmp
        double bodyTilt
        double footX
        double footY
        double supportX
        double supportY
        double turnCompThreshold
        double turnComp

        double velFastForward
        double velFastTurn
        double supportFront
        double supportFront2
        double supportBack
        double supportSideX
        double supportSideY
        double supportTurn

        double frontComp
        double AccelComp

        #Initial body swing
        double supportModYInitial
        double stanceLimitMarginY
        double ankleSupportFaktor

        int pDefault

        Vector2d stanceLimitX
        Vector2d stanceLimitY
        Vector2d stanceLimitA
        Vector3d velDelta
        Vector2d footSizeX
        Vector3d qLArm
        Vector3d qRArm
        Vector2d phSingle
        Vector4d armImuParamX
        Vector4d armImuParamY
        Vector2d zPhase
        Vector2d xPhase

        double default_belly_pitch, default_belly_roll

    _ZMPParameter get_default_parameter()

cdef class ZMPParameter:
    cdef _ZMPParameter* params

cdef class ZMPWalkingEngine:
    cdef _ZMPParameter* parameter
    cdef _ZMPWalk* thisptr
    cpdef init_from_config(self, object zmp_config, string robottype)
    cpdef start(self)
    cpdef stop(self)
    cpdef create_walkready_pose(self, dict config=?, duration=?)
    cpdef process(self)
    cpdef stance_reset(self)
    cpdef set_active(self, bool active)
    cpdef reset(self)
    cpdef set_velocity(self, float x, float y, float z)
    cdef set_gyro(self, int x, int y, int z)
    cdef set_long_legs(self, bool is_long, list legs, list body_offsets, float foot_height)
    cdef set_belly_roll_pid(self, list pid)
