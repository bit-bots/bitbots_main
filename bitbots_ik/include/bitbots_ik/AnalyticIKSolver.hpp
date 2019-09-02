#ifndef ANALYTICIKSOLVER_HPP
#define ANALYTICIKSOLVER_HPP

#include <math.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <moveit/move_group_interface/move_group_interface.h> 
#include <urdf/model.h>

namespace bitbots_ik{
class AnalyticIKSolver{
    public:
        AnalyticIKSolver();
        AnalyticIKSolver(std::string robot_type, const robot_state::JointModelGroup &lleg_joints_group, const robot_state::JointModelGroup &rleg_joints_group);
        bool solve(tf2::Transform& trunk_to_support_foot, tf2::Transform& trunk_to_flying_foot, bool is_left_support, robot_state::RobotStatePtr &robot_state_ptr);
        bool legIK(tf2::Transform& goal, std::vector<double>& positions, bool isLeftLeg, urdf::Model robot);
    private:
        std::string _robot_type;
        const robot_state::JointModelGroup *_lleg_joints_group;
        const robot_state::JointModelGroup *_rleg_joints_group;
};

}

#endif
