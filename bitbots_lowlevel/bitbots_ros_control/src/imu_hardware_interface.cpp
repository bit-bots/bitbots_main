#include <bitbots_ros_control/imu_hardware_interface.hpp>
#include <bitbots_ros_control/utils.hpp>

#define gravity 9.80665

namespace bitbots_ros_control {
using std::placeholders::_1;
using std::placeholders::_2;

ImuHardwareInterface::ImuHardwareInterface(rclcpp::Node::SharedPtr nh, std::shared_ptr<DynamixelDriver> &driver, int id,
                                           std::string topic, std::string frame, std::string name) {
  nh_ = nh;
  driver_ = driver;
  id_ = id;
  topic_ = topic;
  frame_ = frame;
  name_ = name;
  diag_counter_ = 0;
  imu_msg_ = sensor_msgs::msg::Imu();
  imu_msg_.header.frame_id = frame_;
}

bool ImuHardwareInterface::init() {
  status_imu_.name = name_;
  status_imu_.hardware_id = std::to_string(id_);

  imu_pub_ = nh_->create_publisher<sensor_msgs::msg::Imu>(topic_, 10);
  diagnostic_pub_ = nh_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  // read the current values in the IMU module so that they can later be displayed in diagnostic message
  const std::shared_ptr<bitbots_msgs::srv::AccelerometerCalibration::Request> req =
      std::make_shared<bitbots_msgs::srv::AccelerometerCalibration::Request>();
  std::shared_ptr<bitbots_msgs::srv::AccelerometerCalibration::Response> resp =
      std::make_shared<bitbots_msgs::srv::AccelerometerCalibration::Response>();

  write(rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(1e9 * 0));

  return true;
}

void ImuHardwareInterface::read(const rclcpp::Time &t, const rclcpp::Duration &dt) {
  /**
   * Reads the IMU
   */
  bool read_successful = true;
  if (driver_->readMultipleRegisters(id_, 36, 40, data_.data())) {
    // sometimes we only get 0 right after power on, don't use that data
    // test on orientation is sufficient as 0,0,0,0 would not be a valid quaternion
    if (dxlMakeFloat(&data_[24]) + dxlMakeFloat(&data_[28]) + dxlMakeFloat(&data_[32]) + dxlMakeFloat(&data_[36]) !=
        0) {
      angular_velocity_[0] = dxlMakeFloat(&data_[0]);
      angular_velocity_[1] = dxlMakeFloat(&data_[4]);
      angular_velocity_[2] = dxlMakeFloat(&data_[8]);

      linear_acceleration_[0] = dxlMakeFloat(&data_[12]);
      linear_acceleration_[1] = dxlMakeFloat(&data_[16]);
      linear_acceleration_[2] = dxlMakeFloat(&data_[20]);

      orientation_[0] = dxlMakeFloat(&data_[24]);
      orientation_[1] = dxlMakeFloat(&data_[28]);
      orientation_[2] = dxlMakeFloat(&data_[32]);
      orientation_[3] = dxlMakeFloat(&data_[36]);
    }
  } else {
    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "Couldn't read IMU");
    read_successful = false;
  }

  imu_msg_.header.stamp = nh_->get_clock()->now();
  imu_msg_.angular_velocity.x = angular_velocity_[0];
  imu_msg_.angular_velocity.y = angular_velocity_[1];
  imu_msg_.angular_velocity.z = angular_velocity_[2];
  imu_msg_.linear_acceleration.x = linear_acceleration_[0];
  imu_msg_.linear_acceleration.y = linear_acceleration_[1];
  imu_msg_.linear_acceleration.z = linear_acceleration_[2];
  imu_msg_.orientation.x = orientation_[0];
  imu_msg_.orientation.y = orientation_[1];
  imu_msg_.orientation.z = orientation_[2];
  imu_msg_.orientation.w = orientation_[3];
  imu_pub_->publish(imu_msg_);

  // publish diagnostic messages each 100 frames
  if (diag_counter_ % 100 == 0) {
    // diagnostics. check if values are changing, otherwise there is a connection error on the board
    diagnostic_msgs::msg::DiagnosticArray array_msg = diagnostic_msgs::msg::DiagnosticArray();
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> array = std::vector<diagnostic_msgs::msg::DiagnosticStatus>();
    array_msg.header.stamp = nh_->get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status = diagnostic_msgs::msg::DiagnosticStatus();
    // add prefix CORE to sort in diagnostic analyser
    status.name = "IMU" + name_;
    status.hardware_id = std::to_string(id_);
    std::map<std::string, std::string> map;

    if (read_successful) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "OK";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      status.message = "No response";
    }
    std::vector<diagnostic_msgs::msg::KeyValue> keyValues = std::vector<diagnostic_msgs::msg::KeyValue>();
    // itarate through map and save it into values
    for (auto const &ent1 : map) {
      diagnostic_msgs::msg::KeyValue key_value = diagnostic_msgs::msg::KeyValue();
      key_value.key = ent1.first;
      key_value.value = ent1.second;
      keyValues.push_back(key_value);
    }
    status.values = keyValues;
    array.push_back(status);
    array_msg.status = array;
    diagnostic_pub_->publish(array_msg);
  }
  diag_counter_++;
}

void ImuHardwareInterface::write(const rclcpp::Time &t, const rclcpp::Duration &dt) {
}

void ImuHardwareInterface::restoreAfterPowerCycle() { }
}  // namespace bitbots_ros_control
