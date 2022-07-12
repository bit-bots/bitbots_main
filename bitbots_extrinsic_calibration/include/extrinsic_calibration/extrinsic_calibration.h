#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <rot_conv/rot_conv.h>

#include <utility>
using std::placeholders::_1;

class ExtrinsicCalibrationBroadcaster : public rclcpp::Node {
 public:
  ExtrinsicCalibrationBroadcaster();
  void step();
 private:
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  geometry_msgs::msg::Transform transform_{geometry_msgs::msg::Transform()};
  std::string parent_frame_, child_frame_;
  double offset_x_ = 0, offset_y_ = 0, offset_z_ = 0;
  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> &parameters);
};
