#include <bitbots_ros_control/bitfoot_hardware_interface.h>

namespace bitbots_ros_control
{

BitFootHardwareInterface::BitFootHardwareInterface(boost::shared_ptr<DynamixelDriver> driver){
  _driver = driver
  _diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10, true);
  _pressure_pub = nh.advertise<bitbots_msgs::FootPressure>("/foot_pressure", 1, this);

}

bool BitFootHardwareInterface::init(ros::NodeHandle& nh){
  _nh = nh;
  _current_pressure.resize(8, 0);

}

bool BitFootHardwareInterface::read(){
  if (_read_pressure){
    if(!readFootSensors()){
      ROS_ERROR_THROTTLE(1.0, "Couldn't read foot sensor values");
    }
  }
}


bool DynamixelHardwareInterface::readFootSensors(){
  /**
   * Reads the foot pressure sensors of the BitFoots
   */
  uint8_t *data = (uint8_t *) malloc(16 * sizeof(uint8_t));
  // read first foot

  if(_driver->readMultipleRegisters(101, 36, 16, data)){
    for (int i = 0; i < 4; i++) {
      int32_t pres = DXL_MAKEDWORD(DXL_MAKEWORD(data[i*4], data[i*4+1]), DXL_MAKEWORD(data[i*4+2], data[i*4+3]));
      float pres_d = (float) pres;
      // we directly provide raw data since the scaling has to be calibrated by another node for every robot anyway
      _current_pressure[i+4] = (double) pres_d;
    }
  }else{
    ROS_ERROR_THROTTLE(3.0, "Could not read foot with ID 101 (right foot)");
  }
  // read second foot
  if(_driver->readMultipleRegisters(102, 36, 16, data)){
    for (int i = 0; i < 4; i++) {
      int32_t pres = DXL_MAKEDWORD(DXL_MAKEWORD(data[i*4], data[i*4+1]), DXL_MAKEWORD(data[i*4+2], data[i*4+3]));
      float pres_d = (float) pres;
      _current_pressure[i] = (double) pres_d;
    }

  }else{
    ROS_ERROR_THROTTLE(3.0, "Could not read foot with ID 102 (left foot)");
  }

  bitbots_msgs::FootPressure msg;
  msg.header.stamp = ros::Time::now();
  msg.l_l_f = _current_pressure[0];
  msg.l_r_f = _current_pressure[1];
  msg.l_l_b = _current_pressure[2];
  msg.l_r_b = _current_pressure[3];
  msg.r_l_f = _current_pressure[4];
  msg.r_r_f = _current_pressure[5];
  msg.r_l_b = _current_pressure[6];
  msg.r_r_b = _current_pressure[7];
  _pressure_pub.publish(msg);
  return true;
}


// we dont write anything to the pressure sensors
void write(){}

}