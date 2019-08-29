#include <bitbots_ros_control/bitfoot_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control {

BitFootHardwareInterface::BitFootHardwareInterface() {}

BitFootHardwareInterface::BitFootHardwareInterface(std::shared_ptr<DynamixelDriver>& driver, int id, std::string topic_name) {
  _driver = driver;
  _id = id;
  _topic_name = topic_name;
}

bool BitFootHardwareInterface::init(ros::NodeHandle &nh) {
  _nh = nh;
  _current_pressure.resize(4, 0);
  _pressure_pub = nh.advertise<bitbots_msgs::FootPressure>(_topic_name, 1);
  return true;
}

bool BitFootHardwareInterface::read() {
  /**
   * Reads the foot pressure sensors of the BitFoot
   */

  uint8_t *data = (uint8_t *) malloc(16 * sizeof(uint8_t));
  // read foot
  if (_driver->readMultipleRegisters(_id, 36, 16, data)) {
    for (int i = 0; i < 4; i++) {
      int32_t pres = dxl_makedword(dxl_makeword(data[i * 4], data[i * 4 + 1]),
                                   dxl_makeword(data[i * 4 + 2], data[i * 4 + 3]));
      float pres_d = (float) pres;
      // we directly provide raw data since the scaling has to be calibrated by another node for every robot anyway
      _current_pressure[i] = (double) pres_d;
    }
  } else {
    ROS_ERROR_THROTTLE(3.0, "Could not read foot sensor");
  }

  bitbots_msgs::FootPressure msg;
  msg.header.stamp = ros::Time::now();
  msg.left_front = _current_pressure[0];
  msg.right_front = _current_pressure[1];
  msg.left_back = _current_pressure[2];
  msg.right_back= _current_pressure[3];
  _pressure_pub.publish(msg);
  return true;
}

// we dont write anything to the pressure sensors
void BitFootHardwareInterface::write() {}

}
