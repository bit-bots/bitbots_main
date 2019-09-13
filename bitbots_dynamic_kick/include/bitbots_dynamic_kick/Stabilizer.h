#ifndef BITBOTS_DYNAMIC_KICK_STABILIZER_H
#define BITBOTS_DYNAMIC_KICK_STABILIZER_H

#include <optional>
#include <bio_ik/bio_ik.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <bitbots_splines/AbstractStabilizer.h>
#include "KickUtils.h"
#include "Visualizer.h"

class Stabilizer : public AbstractStabilizer<KickPositions> {
public:
    Stabilizer();

    geometry_msgs::Point m_cop_left;
    geometry_msgs::Point m_cop_right;

    /**
     * Calculate required IK goals to reach foot_goal with a foot while keeping the robot as stable as possible.
     * @param positions a description of the required positions
     * @return BioIK Options that can be used by an instance of AbstractIK
     */

    std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> stabilize(const KickPositions& positions) override;
    void reset() override;
    void use_stabilizing(bool use);
    void use_minimal_displacement(bool use);
    void use_cop(bool use);
    void set_trunk_height(double height);
    void set_stabilizing_weight(double weight);
    void set_flying_weight(double weight);
    void set_trunk_orientation_weight(double weight);
    void set_trunk_height_weight(double weight);
    void set_p_factor(double factor_x, double factor_y);
    void set_i_factor(double factor_x, double factor_y);
    void set_d_factor(double factor_x, double factor_y);
    const tf2::Vector3 get_stabilizing_target() const;
    void set_robot_model(moveit::core::RobotModelPtr model);
private:
    moveit::core::RobotModelPtr m_kinematic_model;
    tf2::Vector3 m_stabilizing_target;
    double m_cop_x_error_sum;
    double m_cop_y_error_sum;
    double m_cop_x_error;
    double m_cop_y_error;

    bool m_use_stabilizing;
    bool m_use_minimal_displacement;
    bool m_use_cop;
    double m_trunk_height;
    double m_stabilizing_weight;
    double m_flying_weight;
    double m_trunk_orientation_weight;
    double m_trunk_height_weight;
    double m_p_x_factor;
    double m_p_y_factor;
    double m_i_x_factor;
    double m_i_y_factor;
    double m_d_x_factor;
    double m_d_y_factor;
};

#endif  // BITBOTS_DYNAMIC_KICK_STABILIZER_H
