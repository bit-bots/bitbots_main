#include <bitbots_ros_control/bitfoot_hardware_interface.h>

namespace bitbots_ros_control
{

BitFootHardwareInterface::BitFootHardwareInterface(boost::shared_ptr<DynamixelDriver>&){
  _driver = driver;
}

bool BitFootHardwareInterface::init(ros::NodeHandle& nh){
  _nh = nh;
  _current_pressure.resize(4, 0);
  _diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10, true);
  _pressure_pub = nh.advertise<bitbots_msgs::FootPressure>("/foot_pressure", 1, this);
  //TODO read ID from config

  hardware_interface::FootPressureSensorHandle pressure_handle(_name, _frame, _left_back, _left_front, _right_front, _right_back);
  _pressure_interface.registerHandle(pressure_handle);
  registerInterface(&_pressure_interface);

}

bool BitFootHardwareInterface::read(){
  /**
   * Reads the foot pressure sensors of the BitFoot
   */
  uint8_t *data = (uint8_t *) malloc(16 * sizeof(uint8_t));
  // read first foot

  if(_driver->readMultipleRegisters(_id, 36, 16, data)){
    for (int i = 0; i < 4; i++) {
      int32_t pres = DXL_MAKEDWORD(DXL_MAKEWORD(data[i*4], data[i*4+1]), DXL_MAKEWORD(data[i*4+2], data[i*4+3]));
      float pres_d = (float) pres;
      // we directly provide raw data since the scaling has to be calibrated by another node for every robot anyway
      _current_pressure[i] = (double) pres_d;
    }
  }else{
    ROS_ERROR_THROTTLE(3.0, "Could not read foot sensor with ID %f", _id);
  }

  _left_front = _current_pressure[0];
  _right_front = _current_pressure[1];
  _left_back = _current_pressure[2];
  _right_back = _current_pressure[3];

  return true;
}

// we dont write anything to the pressure sensors
void BitFootHardwareInterface::write() {

}