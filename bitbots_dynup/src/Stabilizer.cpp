#include "bitbots_dynup/Stabilizer.h"
#include "bitbots_dynup/DynamicBalancingGoal.h"
#include "bitbots_dynup/ReferenceGoals.h"

Stabilizer::Stabilizer() {
    /* load MoveIt! model */
    robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
    robot_model_loader.loadKinematicsSolvers(
            kinematics_plugin_loader::KinematicsPluginLoaderPtr(
                    new kinematics_plugin_loader::KinematicsPluginLoader()));

    /* Extract joint groups from loaded model */
    m_kinematic_model = robot_model_loader.getModel();
    m_all_joints_group = m_kinematic_model->getJointModelGroup("All");
    m_legs_joints_group = m_kinematic_model->getJointModelGroup("Legs");

    /* Reset kinematic goal to default */
    m_goal_state.reset(new robot_state::RobotState(m_kinematic_model));
    m_goal_state->setToDefaultValues();

    /* We have to set some good initial position in the goal state,
     * since we are using a gradient based method. Otherwise, the
     * first step will be not correct */
     //TODO
    std::vector<std::string> names_vec = {"HeadPan", "HeadTilt", "LAnklePitch", "LAnkleRoll", "LElbow", "LHipPitch", "LHipRoll", "LHipYaw", "LKnee", "LShoulderPitch", "LShoulderRoll", "RAnklePitch", "RAnkleRoll", "RElbow", "RHipPitch", "RHipRoll", "RHipYaw", "RKnee", "RShoulderPitch", "RShoulderRoll"};
    std::vector<double> pos_vec = {0, 47, -80, 0, -88, 65, -7, 2, -157, 11, -14, 80, 1, 97, -63, -3, 0, 157, -14,  14};

    for (int i = 0; i < names_vec.size(); i++) {
        m_goal_state->setJointPositions(names_vec[i], &pos_vec[i]);
    }
}

std::optional<JointGoals> Stabilizer::stabilize(geometry_msgs::Point support_point, geometry_msgs::PoseStamped& l_foot_goal_pose, geometry_msgs::PoseStamped& trunk_goal_pose) {
    /* ik options is basicaly the command which we send to bio_ik and which describes what we want to do */
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = true;
    double bio_ik_timeout = 0.01;

    tf::Stamped<tf::Pose> tf_l_foot, tf_trunk;
    tf::poseStampedMsgToTF(l_foot_goal_pose, tf_l_foot);
    tf::poseStampedMsgToTF(trunk_goal_pose, tf_trunk);

    /* construct the bio_ik Pose object which tells bio_ik what we want to achieve */
    auto *bio_ik_l_foot_goal = new ReferencePoseGoal();
    bio_ik_l_foot_goal->setPosition(tf_l_foot.getOrigin());
    bio_ik_l_foot_goal->setOrientation(tf_l_foot.getRotation());
    bio_ik_l_foot_goal->setLinkName("l_sole");
    bio_ik_l_foot_goal->setWeight(1.0);
    bio_ik_l_foot_goal->setReferenceLinkName("r_sole");

    auto *bio_ik_trunk_goal = new ReferencePoseGoal();
    bio_ik_trunk_goal->setPosition(tf_trunk.getOrigin());
    bio_ik_trunk_goal->setOrientation(tf_trunk.getRotation());
    bio_ik_trunk_goal->setLinkName("torso");
    bio_ik_trunk_goal->setWeight(1.0);
    bio_ik_trunk_goal->setReferenceLinkName("r_sole");

    tf::Vector3 stabilizing_target = {support_point.x, support_point.y, support_point.z};
    DynamicBalancingContext bio_ik_balancing_context(m_kinematic_model);
    auto *bio_ik_balance_goal = new DynamicBalancingGoal(&bio_ik_balancing_context, stabilizing_target, m_stabilizing_weight);
    bio_ik_balance_goal->setReferenceLink("base_link");


    ik_options.goals.emplace_back(bio_ik_l_foot_goal);
    ik_options.goals.emplace_back(bio_ik_trunk_goal);

    if (m_use_stabilizing && false) {
        ik_options.goals.emplace_back(bio_ik_balance_goal);
    }
    if (m_use_minimal_displacement && false) {
        ik_options.goals.emplace_back(new bio_ik::MinimalDisplacementGoal());
    }

    bool success = m_goal_state->setFromIK(m_legs_joints_group,
                                           EigenSTL::vector_Isometry3d(),
                                           std::vector<std::string>(),
                                           bio_ik_timeout,
                                           moveit::core::GroupStateValidityCallbackFn(),
                                           ik_options);

    if (success) {
        /* retrieve joint names and associated positions from  */
        std::vector<std::string> joint_names = m_legs_joints_group->getActiveJointModelNames();
        std::vector<double> joint_goals;
        m_goal_state->copyJointGroupPositions(m_legs_joints_group, joint_goals);

        /* construct result object */
        JointGoals result;
        result.first = joint_names;
        result.second = joint_goals;
        return result;
    } else {
        return std::nullopt;
    }
}

void Stabilizer::use_stabilizing(bool use) {
    m_use_stabilizing = use;
}

void Stabilizer::use_minimal_displacement(bool use) {
    m_use_minimal_displacement = use;
}

void Stabilizer::set_stabilizing_weight(double weight) {
    m_stabilizing_weight = weight;
}

void Stabilizer::set_flying_weight(double weight) {
    m_flying_weight = weight;
}

void Stabilizer::set_trunk_orientation_weight(double weight) {
    m_trunk_orientation_weight = weight;
}
