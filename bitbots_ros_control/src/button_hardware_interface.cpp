#include <bitbots_ros_control/button_hardware_interface.h>

namespace bitbots_ros_control
{
ButtonHardwareInterface::ButtonHardwareInterface(){}

ButtonHardwareInterface::ButtonHardwareInterface(boost::shared_ptr<DynamixelDriver>& driver){
  _driver = driver;
}

bool ButtonHardwareInterface::init(ros::NodeHandle& nh){
  _nh = nh;
  _button_pub = nh.advertise<bitbots_buttons::Buttons>("/buttons", 1, this);
}

bool ButtonHardwareInterface::read(){
  /**
   * Reads the buttons
   */
  uint8_t *data = (uint8_t *) malloc(sizeof(uint8_t));
  if(_driver->readMultipleRegisters(242, 36, 8, data)){;
    bitbots_buttons::Buttons msg;
    msg.button1 = !(*data & 64);
    msg.button2 = !(*data & 32);
    _button_pub.publish(msg);
    return true;
  }
  ROS_ERROR_THROTTLE(1.0, "Couldn't read Buttons");
  return false;
}

// we dont write anything to the buttons
void ButtonHardwareInterface::write(){}

}