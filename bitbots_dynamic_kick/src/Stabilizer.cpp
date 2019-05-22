#include "bitbots_dynamic_kick/Stabilizer.h"

Stabilizer::Stabilizer() : m_listener(m_tf_buffer) {
    /* load MoveIt! model */
    robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
    robot_model_loader.loadKinematicsSolvers(
            kinematics_plugin_loader::KinematicsPluginLoaderPtr(
                    new kinematics_plugin_loader::KinematicsPluginLoader()));

    /* Extract joint groups from loaded model */
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    m_all_joints_group = kinematic_model->getJointModelGroup("All");
    m_legs_joints_group = kinematic_model->getJointModelGroup("Legs");
    m_lleg_joints_group = kinematic_model->getJointModelGroup("LeftLeg");
    m_rleg_joints_group = kinematic_model->getJointModelGroup("RightLeg");

    /* Reset kinematic goal to default */
    m_goal_state.reset(new robot_state::RobotState(kinematic_model));
    m_goal_state->setToDefaultValues();

    /* We have to set some good initial position in the goal state,
     * since we are using a gradient based method. Otherwise, the
     * first step will be not correct */
    std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
    std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
    for (int i = 0; i < names_vec.size(); i++) {
        m_goal_state->setJointPositions(names_vec[i], &pos_vec[i]);
    }
}

std::optional<JointGoals> Stabilizer::stabilize(bool is_left_kick, geometry_msgs::PoseStamped trunk_goal_lsole, geometry_msgs::PoseStamped flying_foot_goal_lsole) {
    /* ik options is basicaly the command which we send to bio_ik and which describes what we want to do */
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = true;
    double bio_ik_timeout = 0.01;

    // change goals from support foot based coordinate system to trunk based coordinate system
    tf::Transform trunk_goal;
    trunk_goal.setOrigin({trunk_goal_lsole.pose.position.x,
                          trunk_goal_lsole.pose.position.y,
                          trunk_goal_lsole.pose.position.z});
    trunk_goal.setRotation({trunk_goal_lsole.pose.orientation.x,
                            trunk_goal_lsole.pose.orientation.y,
                            trunk_goal_lsole.pose.orientation.z,
                            trunk_goal_lsole.pose.orientation.w});
    tf::Transform support_foot_goal = trunk_goal.inverse();


    tf::Transform flying_foot_goal;
    flying_foot_goal.setOrigin({flying_foot_goal_lsole.pose.position.x,
                                flying_foot_goal_lsole.pose.position.y,
                                flying_foot_goal_lsole.pose.position.z});
    flying_foot_goal.setRotation({flying_foot_goal_lsole.pose.orientation.x,
                                  flying_foot_goal_lsole.pose.orientation.y,
                                  flying_foot_goal_lsole.pose.orientation.z,
                                  flying_foot_goal_lsole.pose.orientation.w});

    flying_foot_goal = support_foot_goal * flying_foot_goal;


    /* construct the bio_ik Pose object which tells bio_ik what we want to achieve */
    bio_ik::PoseGoal* bio_ik_trunk_goal = new bio_ik::PoseGoal();
    bio_ik_trunk_goal->setPosition(support_foot_goal.getOrigin());
    bio_ik_trunk_goal->setOrientation(support_foot_goal.getRotation());
    if(is_left_kick){
        bio_ik_trunk_goal->setLinkName("r_sole");
    } else{
        bio_ik_trunk_goal->setLinkName("l_sole");
    }
    

    bio_ik::PoseGoal* bio_ik_flying_foot_goal = new bio_ik::PoseGoal();
    bio_ik_flying_foot_goal->setPosition(flying_foot_goal.getOrigin());
    bio_ik_flying_foot_goal->setOrientation(flying_foot_goal.getRotation());
    if (is_left_kick) {
        bio_ik_flying_foot_goal->setLinkName("l_sole");
    } else {
        bio_ik_flying_foot_goal->setLinkName("r_sole");
    }

    /* switches order of flying and trunk goal according to is_left_kick */
    if (is_left_kick){
        ik_options.goals.emplace_back(bio_ik_trunk_goal);
    } else {
        ik_options.goals.emplace_back(bio_ik_flying_foot_goal);
    }

    /* call bio_ik on the correct foot to calculate goal_state */
    bool success = m_goal_state->setFromIK(m_rleg_joints_group,
                                           EigenSTL::vector_Isometry3d(),
                                           std::vector<std::string>(),
                                           bio_ik_timeout,
                                           moveit::core::GroupStateValidityCallbackFn(),
                                           ik_options);
    ik_options.goals.clear();

    if (is_left_kick){
        ik_options.goals.emplace_back(bio_ik_flying_foot_goal);
    } else {
        ik_options.goals.emplace_back(bio_ik_trunk_goal);
    }
    success = success && m_goal_state->setFromIK(m_lleg_joints_group,
                                                 EigenSTL::vector_Isometry3d(),
                                                 std::vector<std::string>(),
                                                 bio_ik_timeout,
                                                 moveit::core::GroupStateValidityCallbackFn(),
                                                 ik_options);

    if (success) {
        /* retrieve joint names and associated positions from  */
        std::vector<std::string> joint_names = m_all_joints_group->getActiveJointModelNames();
        std::vector<double> joint_goals;
        m_goal_state->copyJointGroupPositions(m_all_joints_group, joint_goals);

        /* construct result object */
        JointGoals result;
        result.first = joint_names;
        result.second = joint_goals;
        return result;
    } else {
        return std::nullopt;
    }
}
