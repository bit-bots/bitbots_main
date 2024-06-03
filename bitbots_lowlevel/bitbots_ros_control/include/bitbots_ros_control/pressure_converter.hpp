#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/emitter.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <bitbots_msgs/msg/foot_pressure.hpp>
#include <bitbots_msgs/srv/foot_scale.hpp>
#include <fstream>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <iostream>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "pressure_converter_parameters.hpp"

struct FootConfig {
  std::string topic;
  std::vector<double> zero;
  std::vector<double> scale;
  explicit FootConfig(const pressure_converter::Params::Left &foot_config)
      : topic(foot_config.topic), zero(foot_config.zero), scale(foot_config.scale) {}
  explicit FootConfig(const pressure_converter::Params::Right &foot_config)
      : topic(foot_config.topic), zero(foot_config.zero), scale(foot_config.scale) {}
};

class PressureConverter {
 public:
  PressureConverter(rclcpp::Node::SharedPtr nh, pressure_converter::Params::Common config, FootConfig foot_config,
                    char side);

 private:
  rclcpp::Node::SharedPtr nh_;

  // Create publisher and subscriber
  rclcpp::Publisher<bitbots_msgs::msg::FootPressure>::SharedPtr filtered_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cop_pub_;
  std::array<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr, 5> wrench_pubs_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  const char side_;
  const std::string wrench_topics_[5] = {"l_front", "l_back", "r_front", "r_back", "cop"};
  std::array<std::string, 4> wrench_frames_;
  std::array<double, 4> zero_, scale_;
  std::array<std::vector<double>, 4> previous_values_, calibration_buffer_;
  bool save_calibration_buffer_ = false;
  int current_index_ = 0;
  const int average_, calibration_buffer_length_;
  const double cop_threshold_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr zero_service_;
  rclcpp::Service<bitbots_msgs::srv::FootScale>::SharedPtr scale_service_;

  void pressureCallback(bitbots_msgs::msg::FootPressure pressure_raw);
  void resetZeroAndScaleValues();
  bool zeroCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                    std::shared_ptr<std_srvs::srv::Empty::Response> resp);
  bool scaleCallback(const std::shared_ptr<bitbots_msgs::srv::FootScale::Request> req,
                     std::shared_ptr<bitbots_msgs::srv::FootScale::Response> resp);
  void collectMessages();
  void saveYAML();
};
