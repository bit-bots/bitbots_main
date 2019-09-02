#include "bitbots_ik/BioIKSolver.hpp"

namespace bitbots_ik{

BioIKSolver::BioIKSolver(){
    
}

BioIKSolver::BioIKSolver(const robot_state::JointModelGroup &all_joints_group, const robot_state::JointModelGroup &lleg_joints_group, const robot_state::JointModelGroup &rleg_joints_group){
    _use_approximate = true;
    _bioIK_timeout = 0.01;
    _all_joints_group = &all_joints_group;
    _lleg_joints_group = &lleg_joints_group;
    _rleg_joints_group = &rleg_joints_group;
}

bool BioIKSolver::solve(tf2::Transform& trunk_to_support_foot, tf2::Transform& trunk_to_flying_foot, bool is_left_support, robot_state::RobotStatePtr &robot_state_ptr){
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    //ik_options.goals.clear();
    ik_options.replace = true;
    ik_options.return_approximate_solution = _use_approximate;

    //trunk goal
    auto* support_goal = new bio_ik::PoseGoal();
    support_goal->setPosition(trunk_to_support_foot.getOrigin());
    support_goal->setOrientation(trunk_to_support_foot.getRotation());
    if(is_left_support) {
        support_goal->setLinkName("l_sole");
    }else{
        support_goal->setLinkName("r_sole");
    }
    ik_options.goals.emplace_back(support_goal);
    //ik_options.goals.emplace_back(new bio_ik::BalanceGoal());

    // flying foot goal
    auto* fly_goal = new bio_ik::PoseGoal();
    fly_goal->setPosition(trunk_to_flying_foot.getOrigin());
    fly_goal->setOrientation(trunk_to_flying_foot.getRotation());
    if(!is_left_support) {
        fly_goal->setLinkName("l_sole");
    }else{
        fly_goal->setLinkName("r_sole");
    }

    bool success = true;    
    if (is_left_support) {
        success = success && robot_state_ptr->setFromIK(_lleg_joints_group,           // joints which shall be used
                                                EigenSTL::vector_Isometry3d(), // empty and unused by bio-ik
                                                std::vector<std::string>(),  // names of ik links, empty
                                                _bioIK_timeout,                        // timeout
                                                moveit::core::GroupStateValidityCallbackFn(),
                                                ik_options);
        ik_options.goals.clear();
        ik_options.goals.emplace_back(fly_goal);
        //ik_options.goals.emplace_back(new bio_ik::CenterJointsGoal(0.1, true));
        success = success && robot_state_ptr->setFromIK(_rleg_joints_group,           // joints which shall be used
                                                EigenSTL::vector_Isometry3d(), // empty and unused by bio-ik
                                                std::vector<std::string>(),  // names of ik links, empty
                                                _bioIK_timeout,                        // timeout
                                                moveit::core::GroupStateValidityCallbackFn(),
                                                ik_options);
    } else {
        success = success && robot_state_ptr->setFromIK(_rleg_joints_group,           // joints which shall be used
                                                EigenSTL::vector_Isometry3d(), // empty and unused by bio-ik
                                                std::vector<std::string>(),  // names of ik links, empty
                                                _bioIK_timeout,                        // timeout
                                                moveit::core::GroupStateValidityCallbackFn(),
                                                ik_options);
        ik_options.goals.clear();
        ik_options.goals.emplace_back(fly_goal);
        //ik_options.goals.emplace_back(new bio_ik::CenterJointsGoal(0.1, true));
        success = success && robot_state_ptr->setFromIK(_lleg_joints_group,           // joints which shall be used
                                                EigenSTL::vector_Isometry3d(), // empty and unused by bio-ik
                                                std::vector<std::string>(),  // names of ik links, empty
                                                _bioIK_timeout,                        // timeout
                                                moveit::core::GroupStateValidityCallbackFn(),
                                                ik_options);
    }    

    /*balancing*/
    if(false){
        /*tf2::Vector3 balance_target = new tf2::Vector3();
        //todo this should be CoM
        balance_target[0] =0;
        balance_target[1] =0;
        balance_target[2] =0;*/

        auto* balance_goal = new bio_ik::BalanceGoal();
        ik_options.goals.clear();
        ik_options.goals.emplace_back(balance_goal);

        //block the legs
        ik_options.fixed_joints = std::vector<std::string>{"right_ankle_pitch", "right_ankle_roll", "right_knee", "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
                                                            "left_ankle_pitch", "left_ankle_roll", "left_knee", "left_hip_pitch", "left_hip_roll", "left_hip_yaw"};
        success = success && robot_state_ptr->setFromIK(_all_joints_group,           // joints which shall be used
                                                            EigenSTL::vector_Isometry3d(), // empty and unused by bio-ik
                                                            std::vector<std::string>(),  // names of ik links, empty
                                                            _bioIK_timeout,                        // timeout
                                                            moveit::core::GroupStateValidityCallbackFn(),
                                                            ik_options);
    }
     
    return success;

}

void BioIKSolver::set_use_approximate(bool use_approximate){
    _use_approximate = use_approximate;
}

void BioIKSolver::set_bioIK_timeout(double timeout){
    _bioIK_timeout = timeout;
}
}
