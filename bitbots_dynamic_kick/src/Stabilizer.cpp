#include "bitbots_dynamic_kick/Stabilizer.h"
#include "bitbots_dynamic_kick/DynamicBalancingGoal.h"
#include "bitbots_dynamic_kick/ReferenceGoals.h"

Stabilizer::Stabilizer() {
    reset();
}

void Stabilizer::reset() {
    m_cop_x_error_sum = 0.0;
    m_cop_y_error_sum = 0.0;
}

std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> Stabilizer::stabilize(const KickPositions& positions) {
    /* ik options is basically the command which we send to bio_ik and which describes what we want to do */
    auto ik_options = std::make_unique<bio_ik::BioIKKinematicsQueryOptions>();
    ik_options->replace = true;
    ik_options->return_approximate_solution = true;

    tf2::Vector3 stabilizing_target;
    if (positions.cop_support_point && m_use_cop) {
        /* calculate stabilizing target from center of pressure
         * the cop is in corresponding sole frame
         * optimal stabilizing would be centered above sole center */
        double cop_x, cop_y, last_cop_x, last_cop_y, cop_x_error, cop_y_error;
        if (positions.is_left_kick) {
            cop_x = m_cop_right.x;
            cop_y = m_cop_right.y;
        } else {
            cop_x = m_cop_left.x;
            cop_y = m_cop_left.y;
        }
        cop_x_error = cop_x - positions.support_point.x;
        cop_y_error = cop_y - positions.support_point.y;
        m_cop_x_error_sum += cop_x_error;
        m_cop_y_error_sum += cop_y_error;
        stabilizing_target.setX(positions.support_point.x - cop_x * m_p_x_factor - m_i_x_factor * m_cop_x_error_sum - m_d_x_factor * (cop_x_error - m_cop_x_error));
        stabilizing_target.setY(positions.support_point.y - cop_y * m_p_y_factor - m_i_y_factor * m_cop_y_error_sum - m_d_y_factor * (cop_y_error - m_cop_y_error));
        m_cop_x_error = cop_x_error;
        m_cop_y_error = cop_y_error;
        stabilizing_target.setZ(0);
    } else {
        stabilizing_target = {positions.support_point.x, positions.support_point.y, positions.support_point.z};
    }
    m_stabilizing_target = stabilizing_target;

    tf2::Transform flying_foot_goal;
    flying_foot_goal.setOrigin({positions.flying_foot_pose.pose.position.x,
                                positions.flying_foot_pose.pose.position.y,
                                positions.flying_foot_pose.pose.position.z});
    flying_foot_goal.setRotation({positions.flying_foot_pose.pose.orientation.x,
                                  positions.flying_foot_pose.pose.orientation.y,
                                  positions.flying_foot_pose.pose.orientation.z,
                                  positions.flying_foot_pose.pose.orientation.w});


    /* construct the bio_ik Pose object which tells bio_ik what we want to achieve */
    auto *bio_ik_flying_foot_goal = new ReferencePoseGoal();
    bio_ik_flying_foot_goal->setPosition(flying_foot_goal.getOrigin());
    bio_ik_flying_foot_goal->setOrientation(flying_foot_goal.getRotation());
    if (positions.is_left_kick) {
        bio_ik_flying_foot_goal->setLinkName("l_sole");
        bio_ik_flying_foot_goal->setReferenceLinkName("r_sole");
    } else {
        bio_ik_flying_foot_goal->setLinkName("r_sole");
        bio_ik_flying_foot_goal->setReferenceLinkName("l_sole");
    }
    bio_ik_flying_foot_goal->setWeight(m_flying_weight);

    auto *trunk_orientation_goal = new ReferenceOrientationGoal();
    tf2::Quaternion trunk_orientation;
    trunk_orientation.setRPY(0, 0.2, 0);
    trunk_orientation_goal->setOrientation(trunk_orientation);
    trunk_orientation_goal->setLinkName("base_link");
    if (positions.is_left_kick) {
        trunk_orientation_goal->setReferenceLinkName("r_sole");
    } else {
        trunk_orientation_goal->setReferenceLinkName("l_sole");
    }
    trunk_orientation_goal->setWeight(m_trunk_orientation_weight);
    ik_options->goals.emplace_back(trunk_orientation_goal);

    auto *trunk_height_goal = new ReferenceHeightGoal();
    trunk_height_goal->setHeight(m_trunk_height);
    trunk_height_goal->setWeight(m_trunk_height_weight);
    trunk_height_goal->setLinkName("base_link");
    if (positions.is_left_kick) {
        trunk_height_goal->setReferenceLinkName("r_sole");
    } else {
        trunk_height_goal->setReferenceLinkName("l_sole");
    }
    ik_options->goals.emplace_back(trunk_height_goal);

    DynamicBalancingContext* bio_ik_balancing_context = new DynamicBalancingContext(m_kinematic_model);
    auto *bio_ik_balance_goal = new DynamicBalancingGoal(bio_ik_balancing_context, stabilizing_target, m_stabilizing_weight);
    if (positions.is_left_kick) {
        bio_ik_balance_goal->setReferenceLink("r_sole");
    } else {
        bio_ik_balance_goal->setReferenceLink("l_sole");
    }

    ik_options->goals.emplace_back(bio_ik_flying_foot_goal);
    if (m_use_stabilizing) {
        ik_options->goals.emplace_back(bio_ik_balance_goal);
    }
    if (m_use_minimal_displacement) {
        ik_options->goals.emplace_back(new bio_ik::MinimalDisplacementGoal());
    }
    return std::move(ik_options);
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

void Stabilizer::set_p_factor(double factor_x, double factor_y) {
    m_p_x_factor = factor_x;
    m_p_y_factor = factor_y;
}

void Stabilizer::set_i_factor(double factor_x, double factor_y) {
    m_i_x_factor = factor_x;
    m_i_y_factor = factor_y;
}

void Stabilizer::set_d_factor(double factor_x, double factor_y) {
    m_d_x_factor = factor_x;
    m_d_y_factor = factor_y;
}

const tf2::Vector3 Stabilizer::get_stabilizing_target() const {
    return m_stabilizing_target;
}

void Stabilizer::set_robot_model(moveit::core::RobotModelPtr model) {
    m_kinematic_model = model;
}
