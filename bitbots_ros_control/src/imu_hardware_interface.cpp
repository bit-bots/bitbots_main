#include <bitbots_ros_control/imu_hardware_interface.h>

#define gravity 9.80665

namespace bitbots_ros_control
{

ImuHardwareInterface::ImuHardwareInterface(boost::shared_ptr<DynamixelDriver>& driver){
  _driver = driver;
}


bool ImuHardwareInterface::init(ros::NodeHandle& nh){
  _nh = nh;

  //TODO _diagnostic_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10, true);
  _status_IMU.name = "IMU";
  _status_IMU.hardware_id = std::to_string(1);


  // alloc memory for imu values
  _orientation = (double*) malloc(4 * sizeof(double));
  std::fill(_orientation, _orientation+4, 0);
  _orientation_covariance = (double*) malloc(9 * sizeof(double));
  std::fill(_orientation_covariance, _orientation_covariance+9, 0);
  _angular_velocity = (double*) malloc(3 * sizeof(double));
  std::fill(_angular_velocity, _angular_velocity+3, 0);
  _angular_velocity_covariance = (double*) malloc(9 * sizeof(double));
  std::fill(_angular_velocity_covariance, _angular_velocity_covariance+9, 0);
  _linear_acceleration = (double*) malloc(3 * sizeof(double));
  std::fill(_linear_acceleration, _linear_acceleration+3, 0);
  _linear_acceleration_covariance = (double*) malloc(9 * sizeof(double));
  std::fill(_linear_acceleration_covariance, _linear_acceleration_covariance+9, 0);

  // init IMU
  std::string imu_name;
  std::string imu_frame;
  nh.getParam("IMU/name", imu_name);
  nh.getParam("IMU/frame", imu_frame);
  hardware_interface::ImuSensorHandle imu_handle(imu_name, imu_frame, _orientation, _orientation_covariance, _angular_velocity, _angular_velocity_covariance, _linear_acceleration, _linear_acceleration_covariance);
  _imu_interface.registerHandle(imu_handle);
  registerInterface(&_imu_interface);

}

bool ImuHardwareInterface::read(){
  /**
   * Reads the IMU
   */
  uint8_t *data = (uint8_t *) malloc(110 * sizeof(uint8_t));

    if(_driver->readMultipleRegisters(241, 36, 32, data)){
      //todo we have to check if we jumped one sequence number
        uint32_t highest_seq_number = 0;
        uint32_t new_value_index=0;
        uint32_t current_seq_number= 0;
        // imu gives us 2 values at the same time, lets see which one is the newest
        for(int i =0; i < 2; i++){
            //the sequence number are the bytes 12 to 15 for each of the two 16 Bytes
            current_seq_number = DXL_MAKEDWORD(DXL_MAKEWORD(data[16*i+12], data[16*i+13]),
                                             DXL_MAKEWORD(data[16*i+14], data[16*i+15]));
          if(current_seq_number>highest_seq_number){
              highest_seq_number=current_seq_number;
              new_value_index=i;
            }
        }
      // linear acceleration are two signed bytes with 256 LSB per g
      _linear_acceleration[0] = (((short) DXL_MAKEWORD(data[16*new_value_index], data[16*new_value_index+1])) / 256.0 ) * gravity * 1;
      _linear_acceleration[1] = (((short) DXL_MAKEWORD(data[16*new_value_index+2], data[16*new_value_index+3])) / 256.0 ) * gravity * 1;
      _linear_acceleration[2] = (((short)DXL_MAKEWORD(data[16*new_value_index+4], data[16*new_value_index+5])) / 256.0 ) * gravity * 1;
      // angular velocity are two signed bytes with 14.375 per deg/s
      _angular_velocity[0] = (((short)DXL_MAKEWORD(data[16*new_value_index+6], data[16*new_value_index+7])) / 14.375) * M_PI/180 * 1;
      _angular_velocity[1] = (((short)DXL_MAKEWORD(data[16*new_value_index+8], data[16*new_value_index+9])) / 14.375) * M_PI/180 * 1;
      _angular_velocity[2] = (((short)DXL_MAKEWORD(data[16*new_value_index+10], data[16*new_value_index+11])) / 14.375) * M_PI/180 * -1;
      return true;
    }else {
      ROS_ERROR_THROTTLE(1.0, "Couldn't read IMU");
      return false;
    }
}

// we dont write anything to the IMU
void write(){}


}