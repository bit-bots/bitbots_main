//
// Created by judith on 09.03.19.
//

#ifndef BITBOTS_LOCALIZATION_ROBOTSTATE_H
#define BITBOTS_LOCALIZATION_ROBOTSTATE_H

#include <Eigen/Core>
#include <bitbots_localization/tools.hpp>
#include <cmath>
#include <particle_filter/ParticleFilter.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/utils.hpp>
#include <vector>

namespace bitbots_localization {
/**
 * @class RobotState
 * @brief Sample state for a particle filter that localizes the Robot.
 */
class RobotState {
 public:
  RobotState();

  /**
   * @param x Position of the robot.
   * @param y Position of the robot.
   * @param T Orientation of the robot in radians.
   */
  RobotState(double x, double y, double T);

  /**
   * @brief Constructor for a robot state based on a tf2::Transform.
   *
   * @param transform Transform of the robots base_footprint in the map frame.
   */
  explicit RobotState(tf2::Transform transform);

  RobotState operator*(float factor) const;

  RobotState& operator+=(const RobotState& other);

  double getXPos() const;

  double getYPos() const;

  double getTheta() const;

  double getSinTheta() const;

  double getCosTheta() const;

  void setXPos(double x);

  void setYPos(double y);

  void setTheta(double t);

  void setSinTheta(double t);

  void setCosTheta(double t);

  double calcDistance(const RobotState& state) const;

  static void convertParticleListToEigen(const std::vector<particle_filter::Particle<RobotState>*>& particle_list,
                                         Eigen::MatrixXd& matrix, const bool ignore_explorers);

  bool is_explorer_;

  visualization_msgs::msg::Marker renderMarker(std::string n_space, std::string frame, rclcpp::Duration lifetime,
                                               std_msgs::msg::ColorRGBA color, rclcpp::Time stamp) const;

  tf2::Transform getTransform() const;

 private:
  double m_XPos;
  double m_YPos;
  double m_SinTheta;
  double m_CosTheta;
};
};  // namespace bitbots_localization

#endif  // BITBOTS_LOCALIZATION_ROBOTSTATE_H
