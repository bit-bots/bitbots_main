#include <bitbots_ros_control/button_hardware_interface.h>

namespace bitbots_ros_control {
ButtonHardwareInterface::ButtonHardwareInterface(std::shared_ptr<DynamixelDriver> &driver, int id,
                                                 std::string topic, int read_rate) {
  driver_ = driver;
  id_ = id;
  topic_ = topic;
  read_rate_ = read_rate;
}

bool ButtonHardwareInterface::init(ros::NodeHandle &nh, ros::NodeHandle &hw_nh) {
  nh_ = nh;
  data_ = (uint8_t *) malloc(3 * sizeof(uint8_t));
  button_pub_ = nh.advertise<bitbots_buttons::Buttons>(topic_, 1);
  diagnostic_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10, true);

  return true;
}

void ButtonHardwareInterface::read(const ros::Time &t, const ros::Duration &dt) {
  /**
   * Reads the buttons
   */
  counter_ = (counter_ + 1) % read_rate_;
  if (counter_ != 0)
    return;
  bool read_successful = true;
  if (driver_->readMultipleRegisters(id_, 76, 3, data_)) {
    bitbots_buttons::Buttons msg;
    msg.button1 = data_[0];
    msg.button2 = data_[1];
    msg.button3 = data_[2];
    button_pub_.publish(msg);
  } else {
    ROS_ERROR_THROTTLE(1.0, "Couldn't read Buttons");
    read_successful = false;
  }

  // diagnostics. check if values are changing, otherwise there is a connection error on the board
  diagnostic_msgs::DiagnosticArray array_msg = diagnostic_msgs::DiagnosticArray();
  std::vector<diagnostic_msgs::DiagnosticStatus> array = std::vector<diagnostic_msgs::DiagnosticStatus>();
  array_msg.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus status = diagnostic_msgs::DiagnosticStatus();
  // add prefix CORE to sort in diagnostic analyser
  status.name = "BUTTONButton";
  status.hardware_id = std::to_string(id_);

  if (read_successful) {
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "OK";
  } else {
    status.level = diagnostic_msgs::DiagnosticStatus::STALE;
    status.message = "No response";
  }
  array.push_back(status);
  array_msg.status = array;
  diagnostic_pub_.publish(array_msg);
}

// we dont write anything to the buttons
void ButtonHardwareInterface::write(const ros::Time &t, const ros::Duration &dt) {}

}
