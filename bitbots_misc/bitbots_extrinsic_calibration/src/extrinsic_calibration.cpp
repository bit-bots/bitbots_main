#include <rot_conv/rot_conv.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
#include <utility>
using std::placeholders::_1;

class ExtrinsicCalibrationBroadcaster : public rclcpp::Node {
 public:
  ExtrinsicCalibrationBroadcaster() : Node("bitbots_extrinsic_calibration") {
    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    this->declare_parameter<std::string>("parent_frame", "camera_optical_frame_uncalibrated");
    this->declare_parameter<std::string>("child_frame", "camera_optical_frame");
    this->declare_parameter<double>("offset_x", 0.0);
    this->declare_parameter<double>("offset_y", 0.0);
    this->declare_parameter<double>("offset_z", 0.0);

    auto parameters = this->get_parameters(this->list_parameters({}, 10).names);
    onSetParameters(parameters);

    param_callback_handle_ =
        this->add_on_set_parameters_callback(std::bind(&ExtrinsicCalibrationBroadcaster::onSetParameters, this, _1));
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> &parameters) {
    for (const auto &parameter : parameters) {
      if (parameter.get_name() == "offset_x") {
        offset_x_ = parameter.as_double();
      } else if (parameter.get_name() == "offset_y") {
        offset_y_ = parameter.as_double();
      } else if (parameter.get_name() == "offset_z") {
        offset_z_ = parameter.as_double();
      } else if (parameter.get_name() == "parent_frame") {
        parent_frame_ = parameter.as_string();
      } else if (parameter.get_name() == "child_frame") {
        child_frame_ = parameter.as_string();
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Unknown parameter: %s", parameter.get_name().c_str());
      }
    }
    auto offset_quat = rot_conv::QuatFromEuler(offset_z_, offset_y_, offset_x_);

    transform_.rotation.x = offset_quat.x();
    transform_.rotation.y = offset_quat.y();
    transform_.rotation.z = offset_quat.z();
    transform_.rotation.w = offset_quat.w();

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now();
    tf.header.frame_id = parent_frame_;
    tf.child_frame_id = child_frame_;
    tf.transform = transform_;
    broadcaster_->sendTransform(tf);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    return result;
  }

 private:
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  geometry_msgs::msg::Transform transform_{geometry_msgs::msg::Transform()};
  std::string parent_frame_, child_frame_;
  double offset_x_ = 0, offset_y_ = 0, offset_z_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ExtrinsicCalibrationBroadcaster>();

  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
}
