#include "bitbots_dynamic_kick/Stabilizer.h"
#include "bitbots_dynamic_kick/DynamicBalancingGoal.h"
#include "bitbots_dynamic_kick/ReferenceGoals.h"

Stabilizer::Stabilizer() :
        m_cop_error_sum_x(0),
        m_cop_error_sum_y(0),
        m_visualizer("/debug/dynamic_kick") {
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

    reset();

    /* Initialize collision model */
    m_planning_scene.reset(new planning_scene::PlanningScene(m_kinematic_model));
}

void Stabilizer::reset() {
    /* We have to set some good initial position in the goal state,
     * since we are using a gradient based method. Otherwise, the
     * first step will be not correct */
    std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
    std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
    for (int i = 0; i < names_vec.size(); i++) {
        m_goal_state->setJointPositions(names_vec[i], &pos_vec[i]);
    }
    m_cop_error_sum_x = 0.0;
    m_cop_error_sum_y = 0.0;
}

std::optional<JointGoals> Stabilizer::stabilize(bool is_left_kick, geometry_msgs::Point support_point,
        geometry_msgs::PoseStamped flying_foot_goal_pose, bool cop_support_point) {
    /* ik options is basicaly the command which we send to bio_ik and which describes what we want to do */
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.replace = true;
    ik_options.return_approximate_solution = true;
    double bio_ik_timeout = 0.01;

    tf::Vector3 stabilizing_target;
    if (cop_support_point && m_use_cop) {
        /* calculate stabilizing target from center of pressure
         * the cop is in corresponding sole frame
         * optimal stabilizing would be centered above sole center */
        if (is_left_kick) {
            m_cop_error_sum_x += m_cop_right.x - support_point.x;
            m_cop_error_sum_y += m_cop_right.y - support_point.y;
            stabilizing_target.setX(support_point.x - m_cop_right.x * m_p_factor + m_i_factor * m_cop_error_sum_x);
            stabilizing_target.setY(support_point.y - m_cop_right.y * m_p_factor + m_i_factor * m_cop_error_sum_y);
        } else {
            m_cop_error_sum_x += m_cop_left.x - support_point.x;
            m_cop_error_sum_y += m_cop_left.y - support_point.y;
            stabilizing_target.setX(support_point.x - m_cop_left.x * m_p_factor + m_i_factor * m_cop_error_sum_x);
            stabilizing_target.setY(support_point.y - m_cop_left.y * m_p_factor + m_i_factor * m_cop_error_sum_y);
        }
        stabilizing_target.setZ(0);
    } else {
        stabilizing_target = {support_point.x, support_point.y, support_point.z};
    }
    m_visualizer.display_stabilizing_point(stabilizing_target, is_left_kick ? "r_sole" : "l_sole");

    tf::Transform flying_foot_goal;
    flying_foot_goal.setOrigin({flying_foot_goal_pose.pose.position.x,
                                flying_foot_goal_pose.pose.position.y,
                                flying_foot_goal_pose.pose.position.z});
    flying_foot_goal.setRotation({flying_foot_goal_pose.pose.orientation.x,
                                  flying_foot_goal_pose.pose.orientation.y,
                                  flying_foot_goal_pose.pose.orientation.z,
                                  flying_foot_goal_pose.pose.orientation.w});


    /* construct the bio_ik Pose object which tells bio_ik what we want to achieve */
    auto *bio_ik_flying_foot_goal = new ReferencePoseGoal();
    bio_ik_flying_foot_goal->setPosition(flying_foot_goal.getOrigin());
    bio_ik_flying_foot_goal->setOrientation(flying_foot_goal.getRotation());
    if (is_left_kick) {
        bio_ik_flying_foot_goal->setLinkName("l_sole");
        bio_ik_flying_foot_goal->setReferenceLinkName("r_sole");
    } else {
        bio_ik_flying_foot_goal->setLinkName("r_sole");
        bio_ik_flying_foot_goal->setReferenceLinkName("l_sole");
    }
    bio_ik_flying_foot_goal->setWeight(m_flying_weight);

    auto *trunk_orientation_goal = new ReferenceOrientationGoal();
    tf::Quaternion trunk_orientation;
    trunk_orientation.setRPY(0, 0.2, 0);
    trunk_orientation_goal->setOrientation(trunk_orientation);
    trunk_orientation_goal->setLinkName("base_link");
    if (is_left_kick) {
        trunk_orientation_goal->setReferenceLinkName("r_sole");
    } else {
        trunk_orientation_goal->setReferenceLinkName("l_sole");
    }
    trunk_orientation_goal->setWeight(m_trunk_orientation_weight);
    ik_options.goals.emplace_back(trunk_orientation_goal);

    auto *trunk_height_goal = new ReferenceHeightGoal();
    trunk_height_goal->setHeight(m_trunk_height);
    trunk_height_goal->setWeight(m_trunk_height_weight);
    trunk_height_goal->setLinkName("base_link");
    if (is_left_kick) {
        trunk_height_goal->setReferenceLinkName("r_sole");
    } else {
        trunk_height_goal->setReferenceLinkName("l_sole");
    }
    ik_options.goals.emplace_back(trunk_height_goal);

    DynamicBalancingContext bio_ik_balancing_context(m_kinematic_model);
    auto *bio_ik_balance_goal = new DynamicBalancingGoal(&bio_ik_balancing_context, stabilizing_target, m_stabilizing_weight);
    if (is_left_kick) {
        bio_ik_balance_goal->setReferenceLink("r_sole");
    } else {
        bio_ik_balance_goal->setReferenceLink("l_sole");
    }

    ik_options.goals.emplace_back(bio_ik_flying_foot_goal);
    if (m_use_stabilizing) {
        ik_options.goals.emplace_back(bio_ik_balance_goal);
    }
    if (m_use_minimal_displacement) {
        ik_options.goals.emplace_back(new bio_ik::MinimalDisplacementGoal());
    }

    bool success = m_goal_state->setFromIK(m_legs_joints_group,
                                           EigenSTL::vector_Isometry3d(),
                                           std::vector<std::string>(),
                                           bio_ik_timeout,
                                           moveit::core::GroupStateValidityCallbackFn(),
                                           ik_options);

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    collision_detection::AllowedCollisionMatrix acm = m_planning_scene->getAllowedCollisionMatrix();
    m_planning_scene->checkCollision(req, res, *m_goal_state, acm);
    if (res.collision) {
        ROS_ERROR_STREAM("Aborting due to self collision!");
        success = false;
    }

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

void Stabilizer::use_cop(bool use) {
    m_use_cop = use;
}

void Stabilizer::set_trunk_height(double height) {
    m_trunk_height = height;
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

void Stabilizer::set_trunk_height_weight(double weight) {
    m_trunk_height_weight = weight;
}

void Stabilizer::set_p_factor(double factor) {
    m_p_factor = factor;
}

void Stabilizer::set_i_factor(double factor) {
    m_i_factor = factor;
}
