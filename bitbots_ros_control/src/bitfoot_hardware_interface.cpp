#include <bitbots_ros_control/bitfoot_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control {

BitFootHardwareInterface::BitFootHardwareInterface() {}

BitFootHardwareInterface::BitFootHardwareInterface(std::shared_ptr<DynamixelDriver>& driver, int id, std::string topic_name) {
  driver_ = driver;
  id_ = id;
  topic_name_ = topic_name;
}

bool BitFootHardwareInterface::init(ros::NodeHandle &nh) {
  nh_ = nh;
  current_pressure_.resize(4, 0);
  pressure_pub_ = nh.advertise<bitbots_msgs::FootPressure>(topic_name_, 1);
  return true;
}

bool BitFootHardwareInterface::read() {
  /**
   * Reads the foot pressure sensors of the BitFoot
   */

  uint8_t *data = (uint8_t *) malloc(16 * sizeof(uint8_t));
  // read foot
  if (driver_->readMultipleRegisters(id_, 36, 16, data)) {
    for (int i = 0; i < 4; i++) {
      int32_t pres = dxlMakedword(dxlMakeword(data[i*4], data[i*4 + 1]),
                                  dxlMakeword(data[i*4 + 2], data[i*4 + 3]));
      float pres_d = (float) pres;
      // we directly provide raw data since the scaling has to be calibrated by another node for every robot anyway
      current_pressure_[i] = (double) pres_d;
    }
  } else {
    ROS_ERROR_THROTTLE(3.0, "Could not read foot sensor");
  }

  bitbots_msgs::FootPressure msg;
  msg.header.stamp = ros::Time::now();
  msg.left_front = current_pressure_[0];
  msg.right_front = current_pressure_[1];
  msg.left_back = current_pressure_[2];
  msg.right_back= current_pressure_[3];
  pressure_pub_.publish(msg);
  return true;
}

// we dont write anything to the pressure sensors
void BitFootHardwareInterface::write() {}

}
