#include <bitbots_ros_control/leds_hardware_interface.h>
#include <bitbots_ros_control/utils.h>


namespace bitbots_ros_control
{
LedsHardwareInterface::LedsHardwareInterface(){}


LedsHardwareInterface::LedsHardwareInterface(std::shared_ptr<DynamixelDriver>& driver, uint8_t id, uint8_t num_leds){
  driver_ = driver;
  id_ = id;
  leds_.resize(num_leds);
}


bool LedsHardwareInterface::init(ros::NodeHandle& nh, ros::NodeHandle &hw_nh){
  nh_ = nh;
  leds_service_ = nh_.advertiseService("/set_leds", &LedsHardwareInterface::setLeds, this);

  return true;
}

void LedsHardwareInterface::read(const ros::Time& t, const ros::Duration& dt){}

bool LedsHardwareInterface::setLeds(bitbots_msgs::LedsRequest& req, bitbots_msgs::LedsResponse& resp) {
  if (req.leds.size() != leds_.size())
  {
    ROS_WARN_STREAM("You are trying to set "<< req.leds.size() << " leds while the board has " << leds_.size() << " leds.");
    return false;
  }
  for (int i = 0; i < leds_.size(); i++){
    if (req.leds[i].r > 1.0f || req.leds[i].r < 0.0f || 
        req.leds[i].g > 1.0f || req.leds[i].g < 0.0f || 
        req.leds[i].b > 1.0f || req.leds[i].b < 0.0f) {
      ROS_WARN_STREAM("You tried to set LED_" << i << " to a value not between 0 and 1");
      return false;
    }
    leds_[i] = (uint8_t) req.leds[i].r * 255;
    leds_[i] |= ((uint8_t) req.leds[i].g * 255) << 8;
    leds_[i] |= ((uint8_t) req.leds[i].b * 255) << 16;
    leds_[i] |= ((uint8_t) req.leds[i].a * 255) << 24;

  }
  write_leds_ = true;
  return true;
}

void LedsHardwareInterface::write(const ros::Time& t, const ros::Duration& dt) {
  if(write_leds_)
  {

    ROS_INFO("Writing LEDS.");
    driver_->writeRegister(id_, "LED_0", leds_[0]);
    driver_->writeRegister(id_, "LED_1", leds_[1]);
    driver_->writeRegister(id_, "LED_2", leds_[2]);

    write_leds_ = false;
  }
}
}
