#ifndef _POSE_HPP
#define _POSE_HPP

#include <string>
#include <vector>
#include <stdexcept>

#include <boost/interprocess/offset_ptr.hpp>
#include "joint.hpp"

namespace Robot {

/**
 * \brief The pose is the main robot interface to access and manipulate motor positions
 *
 * The Pose is a interprocess shareable object. This represents all the robots motors
 * and positions. The pose offers possibilities to manipulate motor values.
 * Finally the Pose allows access by name, id and defined getting method
 */
class Pose {
public:
    enum Joints{Number = 30};
private:
    Joint j_r_shoulder_pitch;
    Joint j_l_shoulder_pitch;
    Joint j_r_shoulder_roll;
    Joint j_l_shoulder_roll;
    Joint j_r_elbow;
    Joint j_l_elbow;
    Joint j_r_hip_yaw;
    Joint j_l_hip_yaw;
    Joint j_r_hip_roll;
    Joint j_l_hip_roll;
    Joint j_r_hip_pitch;
    Joint j_l_hip_pitch;
    Joint j_r_knee;
    Joint j_l_knee;
    Joint j_r_ankle_pitch;
    Joint j_l_ankle_pitch;
    Joint j_r_ankle_roll;
    Joint j_l_ankle_roll;
    Joint j_head_pan;
    Joint j_head_tilt;
    Joint j_r_elbow_roll;
    Joint j_l_elbow_roll;
    Joint j_r_hand;
    Joint j_l_hand;
    Joint j_r_shoulder_yaw;
    Joint j_l_shoulder_yaw;
    Joint j_r_toe;
    Joint j_l_toe;
    Joint j_belly_roll;
    Joint j_belly_pitch;

    boost::interprocess::offset_ptr<Joint> joints[Joints::Number];

public:
    Pose() {
        joints[0] = &j_r_shoulder_pitch;
        joints[1] = &j_l_shoulder_pitch;
        joints[2] = &j_r_shoulder_roll;
        joints[3] = &j_l_shoulder_roll;
        joints[4] = &j_r_elbow;
        joints[5] = &j_l_elbow;
        joints[6] = &j_r_hip_yaw;
        joints[7] = &j_l_hip_yaw;
        joints[8] = &j_r_hip_roll;
        joints[9] = &j_l_hip_roll;
        joints[10] = &j_r_hip_pitch;
        joints[11] = &j_l_hip_pitch;
        joints[12] = &j_r_knee;
        joints[13] = &j_l_knee;
        joints[14] = &j_r_ankle_pitch;
        joints[15] = &j_l_ankle_pitch;
        joints[16] = &j_r_ankle_roll;
        joints[17] = &j_l_ankle_roll;
        joints[18] = &j_head_pan;
        joints[19] = &j_head_tilt;
        joints[20] = &j_r_elbow_roll;
        joints[21] = &j_l_elbow_roll;
        joints[22] = &j_r_hand;
        joints[23] = &j_l_hand;
        joints[24] = &j_r_shoulder_yaw;
        joints[25] = &j_l_shoulder_yaw;
        joints[26] = &j_r_toe;
        joints[27] = &j_l_toe;
        joints[28] = &j_belly_roll;
        joints[29] = &j_belly_pitch;


        for(int i = 0; i < Joints::Number; i++)
            joints[i]->set_cid(i+1);
    }

    inline Pose& operator=(const Pose& other) {
        j_r_shoulder_pitch = other.j_r_shoulder_pitch;
        j_l_shoulder_pitch = other.j_l_shoulder_pitch;
        j_r_shoulder_roll = other.j_r_shoulder_roll;
        j_l_shoulder_roll = other.j_l_shoulder_roll;
        j_r_elbow = other.j_r_elbow;
        j_l_elbow = other.j_l_elbow;
        j_r_hip_yaw = other.j_r_hip_yaw;
        j_l_hip_yaw = other.j_l_hip_yaw;
        j_r_hip_roll = other.j_r_hip_roll;
        j_l_hip_roll = other.j_l_hip_roll;
        j_r_hip_pitch = other.j_r_hip_pitch;
        j_l_hip_pitch = other.j_l_hip_pitch;
        j_r_knee = other.j_r_knee;
        j_l_knee = other.j_l_knee;
        j_r_ankle_pitch = other.j_r_ankle_pitch;
        j_l_ankle_pitch = other.j_l_ankle_pitch;
        j_r_ankle_roll = other.j_r_ankle_roll;
        j_l_ankle_roll = other.j_l_ankle_roll;
        j_head_pan = other.j_head_pan;
        j_head_tilt = other.j_head_tilt;
        j_r_elbow_roll = other.j_r_elbow_roll;
        j_l_elbow_roll = other.j_l_elbow_roll;
        j_r_hand = other.j_r_hand;
        j_l_hand = other.j_l_hand;
        j_r_shoulder_yaw = other.j_r_shoulder_yaw;
        j_l_shoulder_yaw = other.j_l_shoulder_yaw;
        j_r_toe = other.j_r_toe;
        j_l_toe = other.j_l_toe;
        j_belly_roll = other.j_belly_roll;
        j_belly_pitch = other.j_belly_pitch;

        return *this;
    }

    #define GET_JOINT_IMPL(JOINT) \
    inline \
    Joint& get_ ## JOINT() { \
        return j_ ## JOINT; \
    } \
     \
    inline \
    const Joint& get_ ## JOINT() const { \
        return j_ ## JOINT; \
    }

    GET_JOINT_IMPL(r_shoulder_pitch)
    GET_JOINT_IMPL(l_shoulder_pitch)
    GET_JOINT_IMPL(r_shoulder_roll)
    GET_JOINT_IMPL(l_shoulder_roll)
    GET_JOINT_IMPL(r_elbow)
    GET_JOINT_IMPL(l_elbow)
    GET_JOINT_IMPL(r_hip_yaw)
    GET_JOINT_IMPL(l_hip_yaw)
    GET_JOINT_IMPL(r_hip_roll)
    GET_JOINT_IMPL(l_hip_roll)
    GET_JOINT_IMPL(r_hip_pitch)
    GET_JOINT_IMPL(l_hip_pitch)
    GET_JOINT_IMPL(r_knee)
    GET_JOINT_IMPL(l_knee)
    GET_JOINT_IMPL(r_ankle_pitch)
    GET_JOINT_IMPL(l_ankle_pitch)
    GET_JOINT_IMPL(r_ankle_roll)
    GET_JOINT_IMPL(l_ankle_roll)
    GET_JOINT_IMPL(head_pan)
    GET_JOINT_IMPL(head_tilt)
    GET_JOINT_IMPL(r_elbow_roll)
    GET_JOINT_IMPL(l_elbow_roll)
    GET_JOINT_IMPL(r_hand)
    GET_JOINT_IMPL(l_hand)
    GET_JOINT_IMPL(r_shoulder_yaw)
    GET_JOINT_IMPL(l_shoulder_yaw)
    GET_JOINT_IMPL(r_toe)
    GET_JOINT_IMPL(l_toe)
    GET_JOINT_IMPL(belly_roll)
    GET_JOINT_IMPL(belly_pitch)

    #undef GET_JOINT_IMPL

    inline std::vector<std::string> get_joint_names() const {
        std::vector<std::string> names;

        names.reserve(Joints::Number);

        // Sortiert wie in this->joints!
        names.push_back("RShoulderPitch");
        names.push_back("LShoulderPitch");
        names.push_back("RShoulderRoll");
        names.push_back("LShoulderRoll");
        names.push_back("RElbow");
        names.push_back("LElbow");
        names.push_back("RHipYaw");
        names.push_back("LHipYaw");
        names.push_back("RHipRoll");
        names.push_back("LHipRoll");
        names.push_back("RHipPitch");
        names.push_back("LHipPitch");
        names.push_back("RKnee");
        names.push_back("LKnee");
        names.push_back("RAnklePitch");
        names.push_back("LAnklePitch");
        names.push_back("RAnkleRoll");
        names.push_back("LAnkleRoll");
        names.push_back("HeadPan");
        names.push_back("HeadTilt");
        names.push_back("RElbowRoll");
        names.push_back("LElbowRoll");
        names.push_back("RHand");
        names.push_back("LHand");
        names.push_back("RShoulderYaw");
        names.push_back("LShoulderYaw");
        names.push_back("RToe");
        names.push_back("LToe");
        names.push_back("BellyRoll");
        names.push_back("BellyPitch");

        return names;
    }

    inline std::vector<Joint*> get_joints() {
        #define FILL_JOINT_VECTOR(JOINT_TYPE) \
        std::vector<JOINT_TYPE> joints; \
        joints.reserve(Joints::Number); \
         \
        for(int i = 0; i < Joints::Number; i++) \
            joints.push_back(this->joints[i].get()); \
         \
        return joints;
        FILL_JOINT_VECTOR(Joint*)
    }

    inline std::vector<const Joint*> get_const_joints() const {
        FILL_JOINT_VECTOR(const Joint*)
        #undef FILL_JOINT_VECTOR
    }

    inline void reset() {
        for(int i = 0; i < Joints::Number; i++)
            joints[i]->reset();
    }

    inline void update(const Pose& other) {
        for(unsigned i = 0; i < Joints::Number; i++)
        {
            const Joint& other_joint = *other.joints[i].get();
            if(other_joint.has_changed())
            {
                Joint& joint = *joints[i].get();

                joint.changed = true;
                joint.active = other_joint.is_active();
                joint.goal = other_joint.get_goal();
                joint.speed = other_joint.get_speed();
                joint.p = other_joint.get_p();
            }
            // TODO: winkel übertragen nötig???
            /*
            int tmp = other_joint.get_maximum();
            if (tmp < joint.maximal)
            {
                joint.maximal = tmp;
            }
            tmp = other_joint.get_minimum();
            if (tmp > joint.minimal)
            {
                joint.minimal = tmp;
            }*/
        }
    }

    inline void update_positions(const Pose& other) {
        for(int i = 0; i < Joints::Number; i++) {
            const Joint& joint = *other.joints[i].get();
            joints[i]->set_position(joint.get_position());
        }
    }

    inline Joint* get_joint_ptr(const std::string& name) {
        if(name[0] == 'R') {
            if(name == "RShoulderPitch") return &j_r_shoulder_pitch;
            if(name == "RShoulderRoll") return &j_r_shoulder_roll;
            if(name == "RElbow") return &j_r_elbow;
            if(name == "RHipYaw") return &j_r_hip_yaw;
            if(name == "RHipRoll") return &j_r_hip_roll;
            if(name == "RHipPitch") return &j_r_hip_pitch;
            if(name == "RKnee") return &j_r_knee;
            if(name == "RAnklePitch") return &j_r_ankle_pitch;
            if(name == "RAnkleRoll") return &j_r_ankle_roll;
            if(name == "RElbowRoll") return &j_r_elbow_roll;
            if(name == "RHand") return &j_r_hand;
            if(name == "RShoulderYaw") return &j_r_shoulder_yaw;
            if(name == "RToe") return &j_r_toe;
        }

        if(name[0] == 'L') {
            if(name == "LShoulderPitch") return &j_l_shoulder_pitch;
            if(name == "LShoulderRoll") return &j_l_shoulder_roll;
            if(name == "LElbow") return &j_l_elbow;
            if(name == "LHipYaw") return &j_l_hip_yaw;
            if(name == "LHipRoll") return &j_l_hip_roll;
            if(name == "LHipPitch") return &j_l_hip_pitch;
            if(name == "LKnee") return &j_l_knee;
            if(name == "LAnklePitch") return &j_l_ankle_pitch;
            if(name == "LAnkleRoll") return &j_l_ankle_roll;
            if(name == "LElbowRoll") return &j_l_elbow_roll;
            if(name == "LHand") return &j_l_hand;
            if(name == "LShoulderYaw") return &j_l_shoulder_yaw;
            if(name == "LToe") return &j_l_toe;
        }

        if(name == "HeadPan") return &j_head_pan;
        if(name == "HeadTilt") return &j_head_tilt;
        if(name == "BellyRoll") return &j_belly_roll;
        if(name == "BellyPitch") return &j_belly_pitch;

        throw std::runtime_error(std::string("No joint with this name :") + name);
    }

    inline const Joint* get_joint_by_cid(int cid) const {
        int idx = cid - 1;
        if(idx < 0 || idx >= Joints::Number)
            throw std::runtime_error(std::string("CID out of range! ") + std::to_string(cid));

        return joints[idx].get();
    }

    inline Joint* get_joint_by_cid(int cid) {
        return const_cast<Joint*>(const_cast<const Pose*>(this)->get_joint_by_cid(cid));
    }

    inline void copy(const Pose& other) {
        *this = other;
    }
};

inline constexpr int get_num_joints() {
    return Pose::Joints::Number;
}

} //namespace

#endif

