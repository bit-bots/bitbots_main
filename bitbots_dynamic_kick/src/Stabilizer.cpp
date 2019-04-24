#include "bitbots_dynamic_kick/Stabilizer.h"

Stabilizer::Stabilizer() {
    /* load MoveIt! model */
    robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
    robot_model_loader.loadKinematicsSolvers(
            kinematics_plugin_loader::KinematicsPluginLoaderPtr(
                    new kinematics_plugin_loader::KinematicsPluginLoader()));
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    m_all_joints_group = kinematic_model->getJointModelGroup("All");
    m_legs_joints_group = kinematic_model->getJointModelGroup("Legs");
    m_lleg_joints_group = kinematic_model->getJointModelGroup("LeftLeg");
    m_rleg_joints_group = kinematic_model->getJointModelGroup("RightLeg");
    m_goal_state.reset(new robot_state::RobotState(kinematic_model));
    m_goal_state->setToDefaultValues();

    /* we have to set some good initial position in the goal state,
     * since we are using a gradient based method. Otherwise, the
     * first step will be not correct */
    std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
    std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
    for (int i = 0; i < names_vec.size(); i++) {
        m_goal_state->setJointPositions(names_vec[i], &pos_vec[i]);
    }
}

std::optional<JointGoals> Stabilizer::stabilize(bool is_left_kick, geometry_msgs::Pose foot_goal) {
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = true;
    double bio_ik_timeout = 0.01;

    bio_ik::PoseGoal* bio_ik_foot_goal = new bio_ik::PoseGoal();
    tf::Vector3 position = {foot_goal.position.x, foot_goal.position.y, foot_goal.position.z};

    bio_ik_foot_goal->setPosition(position);
    tf::Quaternion orientation = {foot_goal.orientation.x, foot_goal.orientation.y,
                                  foot_goal.orientation.z, foot_goal.orientation.w};
    bio_ik_foot_goal->setOrientation(orientation);
    if (is_left_kick) {
        bio_ik_foot_goal->setLinkName("l_sole");
    } else {
        bio_ik_foot_goal->setLinkName("r_sole");
    }
    ik_options.goals.emplace_back(bio_ik_foot_goal);

    bool success;
    if (is_left_kick) {
        success = m_goal_state->setFromIK(m_lleg_joints_group,
                                          EigenSTL::vector_Isometry3d(),
                                          std::vector<std::string>(),
                                          bio_ik_timeout,
                                          moveit::core::GroupStateValidityCallbackFn(),
                                          ik_options);
    } else {
        success = m_goal_state->setFromIK(m_rleg_joints_group,
                                          EigenSTL::vector_Isometry3d(),
                                          std::vector<std::string>(),
                                          bio_ik_timeout,
                                          moveit::core::GroupStateValidityCallbackFn(),
                                          ik_options);
    }
    if (success) {
        std::vector<std::string> joint_names = m_all_joints_group->getActiveJointModelNames();
        std::vector<double> joint_goals;
        m_goal_state->copyJointGroupPositions(m_all_joints_group, joint_goals);

        JointGoals result;
        result.first = joint_names;
        result.second = joint_goals;
        return result;
    } else {
        return std::nullopt;
    }
}
