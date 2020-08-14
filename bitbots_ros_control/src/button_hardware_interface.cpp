#include <bitbots_ros_control/button_hardware_interface.h>

namespace bitbots_ros_control
{
ButtonHardwareInterface::ButtonHardwareInterface(std::shared_ptr<DynamixelDriver>& driver, int id, std::string topic){
  driver_ = driver;
  id_ = id;
  topic = topic;
}

bool ButtonHardwareInterface::init(ros::NodeHandle& nh, ros::NodeHandle &hw_nh){
  nh_ = nh;
  button_pub_ = nh.advertise<bitbots_buttons::Buttons>(topic_, 1);
  return true;
}

void ButtonHardwareInterface::read(const ros::Time& t, const ros::Duration& dt){
  /**
   * Reads the buttons
   */
  counter_ = (counter_ + 1) % 100;
  if(counter_ != 0)
    return;
  uint8_t *data = (uint8_t *) malloc(sizeof(uint8_t));
  if(driver_->readMultipleRegisters(241, 76, 3, data)){;
    bitbots_buttons::Buttons msg;
    msg.button1 = data[0];
    msg.button2 = data[1];
    msg.button3 = data[2];
    button_pub_.publish(msg);
  }
  ROS_ERROR_THROTTLE(1.0, "Couldn't read Buttons");
}

// we dont write anything to the buttons
void ButtonHardwareInterface::write(const ros::Time& t, const ros::Duration& dt){}

}
