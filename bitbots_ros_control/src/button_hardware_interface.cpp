#include <bitbots_ros_control/button_hardware_interface.h>

namespace bitbots_ros_control {
ButtonHardwareInterface::ButtonHardwareInterface(rclcpp::Node::SharedPtr nh,
                                                 std::shared_ptr<DynamixelDriver> &driver,
                                                 int id,
                                                 std::string topic,
                                                 int read_rate) {
  nh_ = nh;
  driver_ = driver;
  id_ = id;
  topic_ = topic;
  read_rate_ = read_rate;
}

bool ButtonHardwareInterface::init() {
  data_ = (uint8_t *) malloc(3 * sizeof(uint8_t));
  button_pub_ = nh_->create_publisher<bitbots_msgs::msg::Buttons>(topic_, 1);
  diagnostic_pub_ = nh_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  return true;
}

void ButtonHardwareInterface::read(const rclcpp::Time &t, const rclcpp::Duration &dt) {
  /**
   * Reads the buttons
   */
  counter_ = (counter_ + 1) % read_rate_;
  if (counter_ != 0)
    return;
  bool read_successful = true;
  if (driver_->readMultipleRegisters(id_, 76, 3, data_)) {
    bitbots_msgs::msg::Buttons msg;
    msg.button1 = data_[0];
    msg.button2 = data_[1];
    msg.button3 = data_[2];
    button_pub_->publish(msg);
  } else {
    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000, "Couldn't read Buttons");
    read_successful = false;
  }

  // diagnostics. check if values are changing, otherwise there is a connection error on the board
  diagnostic_msgs::msg::DiagnosticArray array_msg = diagnostic_msgs::msg::DiagnosticArray();
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> array = std::vector<diagnostic_msgs::msg::DiagnosticStatus>();
  array_msg.header.stamp = nh_->get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status = diagnostic_msgs::msg::DiagnosticStatus();
  // add prefix CORE to sort in diagnostic analyser
  status.name = "BUTTONButton";
  status.hardware_id = std::to_string(id_);

  if (read_successful) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    status.message = "No response";
  }
  array.push_back(status);
  array_msg.status = array;
  diagnostic_pub_->publish(array_msg);
}

// we don't write anything to the buttons
void ButtonHardwareInterface::write(const rclcpp::Time &t, const rclcpp::Duration &dt) {}
}
