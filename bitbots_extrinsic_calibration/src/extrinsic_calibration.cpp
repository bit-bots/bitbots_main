#include <extrinsic_calibration/extrinsic_calibration.h>

ExtrinsicCalibrationBroadcaster::ExtrinsicCalibrationBroadcaster() : Node("bitbots_extrinsic_calibration") {

  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  this->declare_parameter<std::string>("parent_frame", "camera");
  this->declare_parameter<std::string>("child_frame", "camera_optical_frame");
  this->declare_parameter<double>("offset_x", 0.0, rcl_interfaces::msg::ParameterDescriptor());
  this->declare_parameter<double>("offset_y", 0.0);
  this->declare_parameter<double>("offset_z", 0.0);

  auto parameters = this->get_parameters(this->list_parameters({}, 10).names);
  ExtrinsicCalibrationBroadcaster::onSetParameters(parameters);

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ExtrinsicCalibrationBroadcaster::onSetParameters, this, _1));
}

rcl_interfaces::msg::SetParametersResult ExtrinsicCalibrationBroadcaster::onSetParameters(
  const std::vector<rclcpp::Parameter> &parameters)
  {

    for (const auto &parameter: parameters) {
      if (parameter.get_name() == "offset_x") {
        offset_x_ = parameter.as_double();
      } else if (parameter.get_name() == "offset_y") {
        offset_y_ = parameter.as_double();
      } else if (parameter.get_name() == "offset_z") {
        offset_z_ = parameter.as_double();
      } else if(parameter.get_name() == "parent_frame") {
        parent_frame_ = parameter.as_string();
      } else if(parameter.get_name() == "child_frame") {
        child_frame_ = parameter.as_string();
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Unknown parameter: %s", parameter.get_name().c_str());
      }
    }

    auto base_quat = rot_conv::QuatFromEuler(-1.5708, 0.0, -1.5708);
    auto offset_quat = rot_conv::QuatFromEuler(offset_z_, offset_y_, offset_x_);

    auto final_quat = offset_quat * base_quat;

    transform_.rotation.x = final_quat.x();
    transform_.rotation.y = final_quat.y();
    transform_.rotation.z = final_quat.z();
    transform_.rotation.w = final_quat.w();

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    return result;
  }


void ExtrinsicCalibrationBroadcaster::step() {
  auto node_pointer = this->shared_from_this();
  rclcpp::Time now = this->now();

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now;
  tf.header.frame_id = parent_frame_;
  tf.child_frame_id = child_frame_;
  tf.transform = transform_;
  broadcaster_->sendTransform(tf);
  RCLCPP_ERROR(this->get_logger(), "Send");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ExtrinsicCalibrationBroadcaster>();

  rclcpp::TimerBase::SharedPtr timer = rclcpp::create_timer(
    node, node->get_clock(), rclcpp::Duration(0, 1e7), [node]() -> void {node->step();});

  rclcpp::executors::EventsExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
}

