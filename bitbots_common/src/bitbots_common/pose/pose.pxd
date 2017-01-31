from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp cimport bool

cdef import from "joint.hpp":
    cdef cppclass Joint "Robot::Joint":
        void set_active(bool active)
        bool set_goal(float goal)
        void set_speed(float speed)
        void set_position(float position)
        void set_load(float load)
        void set_temperature(float temperature)
        void set_voltage(float voltage)
        void reset()


        void set_p(int p)
        int get_p()

        void set_i(int i)
        int get_i()

        void set_d(int d)
        int get_d()

        void set_maximum(float maximum)
        void set_minimum(float minimum)

        bool is_active()
        bool has_changed()
        float get_goal()
        float get_speed()
        float get_position()
        float get_temperature()
        float get_voltage()
        float get_load()
        int get_cid()
        float get_maximum()
        float get_minimum()

cdef extern from "pose.hpp" namespace "Robot":
    int get_num_joints()

    cdef cppclass Pose:
        void copy(Pose& other)

        vector[string] get_joint_names()
        vector[Joint*] get_joints()

        Joint* get_joint_ptr(string& name) except +
        Joint* get_joint_by_cid(int cid) except +

        Joint& get_r_shoulder_pitch()
        Joint& get_l_shoulder_pitch()
        Joint& get_r_shoulder_roll()
        Joint& get_l_shoulder_roll()
        Joint& get_r_elbow()
        Joint& get_l_elbow()
        Joint& get_r_hip_yaw()
        Joint& get_l_hip_yaw()
        Joint& get_r_hip_roll()
        Joint& get_l_hip_roll()
        Joint& get_r_hip_pitch()
        Joint& get_l_hip_pitch()
        Joint& get_r_knee()
        Joint& get_l_knee()
        Joint& get_r_ankle_pitch()
        Joint& get_l_ankle_pitch()
        Joint& get_r_ankle_roll()
        Joint& get_l_ankle_roll()
        Joint& get_head_pan()
        Joint& get_head_tilt()
        Joint& get_l_hand()
        Joint& get_r_hand()
        Joint& get_l_elbow_roll()
        Joint& get_r_elbow_roll()
        Joint& get_r_shoulder_yaw()
        Joint& get_l_shoulder_yaw()
        Joint& get_r_toe()
        Joint& get_l_toe()
        Joint& get_belly_roll()
        Joint& get_belly_pitch()

        void reset()
        void update(Pose& other)
        void update_positions(Pose& other)
