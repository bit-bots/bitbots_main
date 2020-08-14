#include <bitbots_ros_control/imu_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

#define gravity 9.80665

namespace bitbots_ros_control
{

ImuHardwareInterface::ImuHardwareInterface(std::shared_ptr<DynamixelDriver>& driver, int id, std::string topic, std::string frame, std::string name){
  driver_ = driver;
  id_ = id;
  topic_ = topic;
  frame_ = frame;
  name_ = name;
}


bool ImuHardwareInterface::init(ros::NodeHandle& nh, ros::NodeHandle &hw_nh){
  nh_ = nh;
  status_imu_.name = name_;
  status_imu_.hardware_id = std::to_string(id_);


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

  data_ = (uint8_t *) malloc(40 * sizeof(uint8_t));
  accel_calib_data_ = (uint8_t *) malloc(28 * sizeof(uint8_t));
  
  // init IMU
  hardware_interface::ImuSensorHandle imu_handle(topic_, frame_, orientation_, orientation_covariance_, angular_velocity_, angular_velocity_covariance_, linear_acceleration_, linear_acceleration_covariance_);
  imu_interface_.registerHandle(imu_handle);
  parent_->registerInterface(&imu_interface_);

  // make services
  imu_ranges_service_ = nh_.advertiseService("/imu/set_imu_ranges", &ImuHardwareInterface::setIMURanges, this);
  calibrate_gyro_service_ = nh_.advertiseService("/imu/calibrate_gyro", &ImuHardwareInterface::calibrateGyro, this);
  reset_gyro_calibration_service_ = nh_.advertiseService("/imu/reset_gyro_calibration", &ImuHardwareInterface::resetGyroCalibration, this);
  complementary_filter_params_service_ = nh_.advertiseService("/imu/set_complementary_filter_params", &ImuHardwareInterface::setComplementaryFilterParams, this);
  calibrate_accel_service_ = nh_.advertiseService("/imu/calibrate_accel", &ImuHardwareInterface::calibrateAccel, this);
  read_accel_calibration_service_ = nh_.advertiseService("/imu/read_accel_calibration", &ImuHardwareInterface::readAccelCalibration, this);
  reset_accel_calibration_service_ = nh_.advertiseService("/imu/reset_accel_calibration", &ImuHardwareInterface::resetAccelCalibraton, this);
  set_accel_calib_threshold_service_ = nh_.advertiseService("/imu/set_accel_calibration_threshold", &ImuHardwareInterface::setAccelCalibrationThreshold, this);

  uint16_t model_number = uint16_t(0xbaff);
  uint16_t* model_number_p = &model_number;
  if(driver_->ping(id_, model_number_p))
    ROS_WARN_STREAM("PING IMU SUCESSFUL, MODEL_NUM: " << *model_number_p);
  else
  {
    ROS_ERROR("IMU could not be pinged");
  }
  
  return true;
}

void ImuHardwareInterface::read(const ros::Time& t, const ros::Duration& dt){
  /**
   * Reads the IMU
   */

  if(driver_->readMultipleRegisters(id_, 36, 40, data_)){
    angular_velocity_[0] = dxlMakeFloat(data_ + 0);
    angular_velocity_[1] = dxlMakeFloat(data_ + 4);
    angular_velocity_[2] = dxlMakeFloat(data_ + 8);

    linear_acceleration_[0] = dxlMakeFloat(data_ + 12);
    linear_acceleration_[1] = dxlMakeFloat(data_ + 16);
    linear_acceleration_[2] = dxlMakeFloat(data_ + 20);

    orientation_[0] = dxlMakeFloat(data_ + 24);
    orientation_[1] = dxlMakeFloat(data_ + 28);
    orientation_[2] = dxlMakeFloat(data_ + 32);
    orientation_[3] = dxlMakeFloat(data_ + 36);
  }else {
    ROS_ERROR_THROTTLE(1.0, "Couldn't read IMU");
  }
}

bool ImuHardwareInterface::setIMURanges(bitbots_msgs::IMURangesRequest& req, bitbots_msgs::IMURangesResponse& resp) {
  accel_range_ = req.accel_range;
  gyro_range_ = req.gyro_range;
  write_ranges_ = true;
  return true;
}

bool ImuHardwareInterface::calibrateGyro(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp) {
  calibrate_gyro_ = true;
  return true;
}

bool ImuHardwareInterface::resetGyroCalibration(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp) {
  reset_gyro_calibration_ = true;
  return true;
}

bool ImuHardwareInterface::setComplementaryFilterParams(bitbots_msgs::ComplementaryFilterParamsRequest& req,
                                                        bitbots_msgs::ComplementaryFilterParamsResponse& resp) {

  do_adaptive_gain_ = req.do_adaptive_gain;
  do_bias_estimation_ = req.do_bias_estimation;
  accel_gain_ = req.accel_gain;
  bias_alpha_ = req.bias_alpha;
  write_complementary_filter_params_ = true;
  return true;
}

bool ImuHardwareInterface::calibrateAccel(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
  calibrate_accel_ = true;
  return true;
}

bool ImuHardwareInterface::resetAccelCalibraton(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
  reset_accel_calibration_ = true;
  return true;
}

bool ImuHardwareInterface::readAccelCalibration(bitbots_msgs::AccelerometerCalibrationRequest& req, bitbots_msgs::AccelerometerCalibrationResponse& resp)
{
  resp.biases.resize(3);
  resp.scales.resize(3);
  if(driver_->readMultipleRegisters(id_, 118, 28, accel_calib_data_))
  {
    resp.threshold = dxlMakeFloat(accel_calib_data_ + 0);
    resp.biases[0] = dxlMakeFloat(accel_calib_data_ + 4);
    resp.biases[1] = dxlMakeFloat(accel_calib_data_ + 8);
    resp.biases[2] = dxlMakeFloat(accel_calib_data_ + 12);
    resp.scales[0] = dxlMakeFloat(accel_calib_data_ + 16);
    resp.scales[1] = dxlMakeFloat(accel_calib_data_ + 20);
    resp.scales[2] = dxlMakeFloat(accel_calib_data_ + 24);
    return true;
  }
  else
  {
    return false;
  }
}

bool ImuHardwareInterface::setAccelCalibrationThreshold(bitbots_msgs::SetAccelerometerCalibrationThresholdRequest& req,
                                                        bitbots_msgs::SetAccelerometerCalibrationThresholdResponse& resp)
{
  accel_calib_threshold_ = req.threshold;
  set_accel_calib_threshold_ = true;
  return true;
}

void ImuHardwareInterface::write(const ros::Time& t, const ros::Duration& dt) {
  if(write_ranges_) {
    ROS_INFO_STREAM("Setting Gyroscope range to " << gyroRangeToString(gyro_range_));
    ROS_INFO_STREAM("Setting Accelerometer range to " << accelRangeToString(accel_range_));
    driver_->writeRegister(id_, "Gyro_Range", gyro_range_);
    driver_->writeRegister(id_, "Accel_Range", accel_range_);
    write_ranges_ = false;
  }
  if(calibrate_gyro_){
    ROS_INFO("Calibrating gyroscope");
    driver_->writeRegister(id_, "Calibrate_Gyro", 1);
    calibrate_gyro_ = false;
  }
  if(reset_gyro_calibration_){
    ROS_INFO("Resetting gyroscope Calibration");
    driver_->writeRegister(id_, "Reset_Gyro_Calibration", 1);
    reset_gyro_calibration_ = false;
  }
  if(write_complementary_filter_params_){
    ROS_INFO("Writing Complementary Filter parameters.");
    driver_->writeRegister(id_, "Do_Adaptive_Gain", do_adaptive_gain_);
    driver_->writeRegister(id_, "Do_Bias_Estimation", do_bias_estimation_);
    driver_->writeRegister(id_, "Accel_Gain", accel_gain_);
    driver_->writeRegister(id_, "Bias_Alpha", bias_alpha_);
    write_complementary_filter_params_ = false;
  }
  if(calibrate_accel_)
  {
    driver_->writeRegister(id_, "Calibrate_Accel", 1);
    calibrate_accel_ = false;
  }
  if(reset_accel_calibration_)
  {
    driver_->writeRegister(id_, "Reset_Accel_Calibration", 1);
    reset_accel_calibration_ = false;
  }
  if(set_accel_calib_threshold_)
  {
    uint32_t data;
    memcpy(&data, &accel_calib_threshold_, sizeof(data));
    driver_->writeRegister(id_, "Accel_Calibration_Threshold", data);
    set_accel_calib_threshold_ = false;
  }
}

void ImuHardwareInterface::setParent(hardware_interface::RobotHW* parent) {
  parent_ = parent;
}

}


