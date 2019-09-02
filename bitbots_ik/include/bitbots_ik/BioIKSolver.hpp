#ifndef BIOIKSOLVER_HPP
#define BIOIKSOLVER_HPP

#include "ros/ros.h"
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

#include "bio_ik/bio_ik.h"
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h> //kinetic
//#include <moveit/move_group_interface/move_group.h> // INDIGO, JADE
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

namespace bitbots_ik{

class BioIKSolver{
    public:
        BioIKSolver();
        BioIKSolver(const robot_state::JointModelGroup &all_joints_group, const robot_state::JointModelGroup &lleg_joints_group, const robot_state::JointModelGroup &rleg_joints_group);
        bool solve(tf2::Transform& trunk_to_support_foot, tf2::Transform& trunk_to_flying_foot, bool is_left_support, robot_state::RobotStatePtr &robot_state_ptr);
        void set_use_approximate(bool use_approximate);
        void set_bioIK_timeout(double timeout);
    private:
        bool _use_approximate;
        double _bioIK_timeout;
        const robot_state::JointModelGroup *_all_joints_group;
        const robot_state::JointModelGroup *_lleg_joints_group;
        const robot_state::JointModelGroup *_rleg_joints_group;

};
}

#endif
