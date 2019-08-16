#include <bitbots_ros_control/bitfoot_hardware_interface.h>
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))

namespace bitbots_ros_control {

BitFootHardwareInterface::BitFootHardwareInterface(boost::shared_ptr<DynamixelDriver> &, int id, std::string topic_name) {
  _driver = driver;
  _id = id;
  _topic_name = topic_name;
}

bool BitFootHardwareInterface::init(ros::NodeHandle &nh) {
  _nh = nh;
  _current_pressure.resize(4, 0);
  //TODO _diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10, true);
  _pressure_pub = nh.advertise<bitbots_msgs::FootPressure>(_topic_name, 1, this);
}

bool BitFootHardwareInterface::read() {
  /**
   * Reads the foot pressure sensors of the BitFoot
   */

  uint8_t *data = (uint8_t *) malloc(16 * sizeof(uint8_t));
  // read first foot
  if (_driver->readMultipleRegisters(_id, 36, 16, data)) {
    for (int i = 0; i < 4; i++) {
      int32_t pres = DXL_MAKEDWORD(DXL_MAKEWORD(data[i * 4], data[i * 4 + 1]),
                                   DXL_MAKEWORD(data[i * 4 + 2], data[i * 4 + 3]));
      float pres_d = (float) pres;
      // we directly provide raw data since the scaling has to be calibrated by another node for every robot anyway
      _current_pressure[i] = (double) pres_d;
    }
  } else {
    ROS_ERROR_THROTTLE(3.0, "Could not read foot with ID %f", _id);
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