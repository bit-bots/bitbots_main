#include <bitbots_ros_control/button_hardware_interface.h>

namespace bitbots_ros_control
{
ButtonHardwareInterface::ButtonHardwareInterface(){}

ButtonHardwareInterface::ButtonHardwareInterface(std::shared_ptr<DynamixelDriver>& driver){
  driver_ = driver;
}

bool ButtonHardwareInterface::init(ros::NodeHandle& nh){
  nh_ = nh;
  button_pub_ = nh.advertise<bitbots_buttons::Buttons>("/buttons", 1);
  return true;
}

bool ButtonHardwareInterface::read(){
  /**
   * Reads the buttons
   */
  uint8_t *data = (uint8_t *) malloc(sizeof(uint8_t));
  if(driver_->readMultipleRegisters(242, 36, 8, data)){;
    bitbots_buttons::Buttons msg;
    msg.button1 = !(*data & 64);
    msg.button2 = !(*data & 32);
    button_pub_.publish(msg);
    return true;
  }
  ROS_ERROR_THROTTLE(1.0, "Couldn't read Buttons");
  return false;
}

// we dont write anything to the buttons
void ButtonHardwareInterface::write(){}

}
