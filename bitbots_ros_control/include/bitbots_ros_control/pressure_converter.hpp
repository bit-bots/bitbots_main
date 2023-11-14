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

class PressureConverter {
 public:
  PressureConverter(rclcpp::Node::SharedPtr nh, char side);

 private:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::executors::StaticSingleThreadedExecutor sub_executor_;
  rclcpp::CallbackGroup::SharedPtr sub_cbg_;
  std::thread* sub_executor_thread_;
  rclcpp::Publisher<bitbots_msgs::msg::FootPressure>::SharedPtr filtered_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cop_pub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> wrench_pubs_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<std::string> wrench_frames_;
  std::vector<double> zero_, scale_;
  std::vector<std::vector<double>> previous_values_, zero_and_scale_values_;
  bool save_zero_and_scale_values_;
  int current_index_;
  int average_, scale_and_zero_average_;
  double cop_threshold_;
  char side_;
  std::string req_type_;
  std::string scale_lr_, zero_lr_, cop_lr_, sole_lr_;
  std::shared_ptr<bitbots_msgs::srv::FootScale::Request> request_;

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
