#include <bitbots_ros_control/imu_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

#define gravity 9.80665

namespace bitbots_ros_control
{
ImuHardwareInterface::ImuHardwareInterface(){}


ImuHardwareInterface::ImuHardwareInterface(std::shared_ptr<DynamixelDriver>& driver){
  driver_ = driver;
}


bool ImuHardwareInterface::init(ros::NodeHandle& nh){
  nh_ = nh;

  status_imu_.name = "IMU";
  status_imu_.hardware_id = std::to_string(1);


  // alloc memory for imu values
  orientation_ = (double*) malloc(4 * sizeof(double));
  std::fill(orientation_, orientation_+4, 0);
  orientation_covariance_ = (double*) malloc(9 * sizeof(double));
  std::fill(orientation_covariance_, orientation_covariance_+9, 0);
  angular_velocity_ = (double*) malloc(3 * sizeof(double));
  std::fill(angular_velocity_, angular_velocity_+3, 0);
  angular_velocity_covariance_ = (double*) malloc(9 * sizeof(double));
  std::fill(angular_velocity_covariance_, angular_velocity_covariance_+9, 0);
  linear_acceleration_ = (double*) malloc(3 * sizeof(double));
  std::fill(linear_acceleration_, linear_acceleration_+3, 0);
  linear_acceleration_covariance_ = (double*) malloc(9 * sizeof(double));
  std::fill(linear_acceleration_covariance_, linear_acceleration_covariance_+9, 0);

  // init IMU
  std::string imu_name;
  std::string imu_frame;
  nh.getParam("IMU/name", imu_name);
  nh.getParam("IMU/frame", imu_frame);
  hardware_interface::ImuSensorHandle imu_handle(imu_name, imu_frame, orientation_, orientation_covariance_, angular_velocity_, angular_velocity_covariance_, linear_acceleration_, linear_acceleration_covariance_);
  imu_interface_.registerHandle(imu_handle);
  parent_->registerInterface(&imu_interface_);

  return true;
}

bool ImuHardwareInterface::read(){
  /**
   * Reads the IMU
   */
  uint8_t *data = (uint8_t *) malloc(110 * sizeof(uint8_t));

    if(driver_->readMultipleRegisters(241, 36, 32, data)){
        uint32_t highest_seq_number = 0;
        uint32_t new_value_index=0;
        uint32_t current_seq_number= 0;
        // imu gives us 2 values at the same time, lets see which one is the newest
        for(int i =0; i < 2; i++){
            //the sequence number are the bytes 12 to 15 for each of the two 16 Bytes
            current_seq_number = dxlMakedword(dxlMakeword(data[16*i + 12], data[16*i + 13]),
                                              dxlMakeword(data[16*i + 14], data[16*i + 15]));
          if(current_seq_number>highest_seq_number){
              highest_seq_number=current_seq_number;
              new_value_index=i;
            }
        }
      // linear acceleration are two signed bytes with 256 LSB per g
      linear_acceleration_[0] = (((short) dxlMakeword(data[16*new_value_index], data[16*new_value_index + 1])) / 256.0 ) * gravity * -1;
      linear_acceleration_[1] = (((short) dxlMakeword(data[16*new_value_index + 2], data[16*new_value_index + 3])) / 256.0 ) * gravity * -1;
      linear_acceleration_[2] = (((short) dxlMakeword(data[16*new_value_index + 4], data[16*new_value_index + 5])) / 256.0 ) * gravity * 1;
      // angular velocity are two signed bytes with 14.375 per deg/s
      angular_velocity_[0] = (((short) dxlMakeword(data[16*new_value_index + 6], data[16*new_value_index + 7])) / 14.375) * M_PI/180 * -1;
      angular_velocity_[1] = (((short) dxlMakeword(data[16*new_value_index + 8], data[16*new_value_index + 9])) / 14.375) * M_PI/180 * -1;
      angular_velocity_[2] = (((short) dxlMakeword(data[16*new_value_index + 10], data[16*new_value_index + 11])) / 14.375) * M_PI/180 * 1;
      return true;
    }else {
      ROS_ERROR_THROTTLE(1.0, "Couldn't read IMU");
      return false;
    }
}

// we dont write anything to the IMU
void ImuHardwareInterface::write(){}

void ImuHardwareInterface::setParent(hardware_interface::RobotHW* parent) {
  parent_ = parent;
}

}
