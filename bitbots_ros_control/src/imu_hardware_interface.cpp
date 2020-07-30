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

  // make services
  imu_ranges_service_ = nh.advertiseService("/imu/set_imu_ranges", &ImuHardwareInterface::setIMURanges, this);
  //calibrate_gyro_service_ = pnh.advertiseService(+ "/set_gyro_range", &ImuHardwareInterface::scaleCallback, this);
  //reset_gyro_calibration_service_ = pnh.advertiseService("/set_gyro_range", &ImuHardwareInterface::scaleCallback, this);
  //complementary_filter_params_service_ = pnh.advertiseService("/set_complementary_filter_params", &ImuHardwareInterface::scaleCallback, this);

  return true;
}

bool ImuHardwareInterface::read(){
  /**
   * Reads the IMU
   */
  uint8_t *data = (uint8_t *) malloc(40 * sizeof(uint8_t));

    if(driver_->readMultipleRegisters(241, 36, 40, data)){
      angular_velocity_[0] = dxlMakeFloat(data + 0);
      angular_velocity_[1] = dxlMakeFloat(data + 4);
      angular_velocity_[2] = dxlMakeFloat(data + 8);

      linear_acceleration_[0] = dxlMakeFloat(data + 12);
      linear_acceleration_[1] = dxlMakeFloat(data + 16);
      linear_acceleration_[2] = dxlMakeFloat(data + 20);

      orientation_[0] = dxlMakeFloat(data + 24);
      orientation_[1] = dxlMakeFloat(data + 28);
      orientation_[2] = dxlMakeFloat(data + 32);
      orientation_[3] = dxlMakeFloat(data + 36);
      return true;
    }else {
      ROS_ERROR_THROTTLE(1.0, "Couldn't read IMU");
      return false;
    }
}
/*
#define ADDR_CONTROL_ITEM_GYRO_RANGE 102
#define ADDR_CONTROL_ITEM_ACCEL_RANGE 103
#define ADDR_CONTROL_ITEM_CALIBRATE_GYRO 104
#define ADDR_CONTROL_ITEM_RESET_GYRO_CALIBRATION 105

#define ADDR_CONTROL_ITEM_DO_ADAPTIVE_GAIN 108
#define ADDR_CONTROL_ITEM_DO_BIAS_ESTIMATION 109
#define ADDR_CONTROL_ITEM_GAIN 110
#define ADDR_CONTROL_ITEM_ALPHA 114
*/
bool ImuHardwareInterface::setIMURanges(bitbots_msgs::IMURangesRequest& req, bitbots_msgs::IMURangesResponse& resp) {
  accel_range_ = req.accel_range;
  gyro_range_ = req.gyro_range;
  write_ranges_ = true;
  return true;
}
bool ImuHardwareInterface::calibrateGyro(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp) {
  write_calibrate_gyro_ = true;
  return true;
}

void ImuHardwareInterface::write() {
  if(write_ranges_) {
    ROS_WARN("writing ranges");
    write_ranges_ = false;
  }
  if(write_calibrate_gyro_){
    ROS_WARN("Calibrating gyro");
  }
  if(write_reset_gyro_calibration_){
    ROS_WARN("resetting gyro calib");
  }

}

void ImuHardwareInterface::setParent(hardware_interface::RobotHW* parent) {
  parent_ = parent;
}

}
