#include <bitbots_ros_control/imu_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

#define gravity 9.80665

namespace bitbots_ros_control {

ImuHardwareInterface::ImuHardwareInterface(std::shared_ptr<DynamixelDriver> &driver,
                                           int id,
                                           std::string topic,
                                           std::string frame,
                                           std::string name) {
  driver_ = driver;
  id_ = id;
  topic_ = topic;
  frame_ = frame;
  name_ = name;
  diag_counter_ = 0;
}

bool ImuHardwareInterface::init(ros::NodeHandle &nh, ros::NodeHandle &hw_nh) {
  nh_ = nh;
  status_imu_.name = name_;
  status_imu_.hardware_id = std::to_string(id_);


  // alloc memory for imu values
  orientation_ = (double *) malloc(4 * sizeof(double));
  std::fill(orientation_, orientation_ + 4, 0);
  orientation_covariance_ = (double *) malloc(9 * sizeof(double));
  std::fill(orientation_covariance_, orientation_covariance_ + 9, 0);
  angular_velocity_ = (double *) malloc(3 * sizeof(double));
  std::fill(angular_velocity_, angular_velocity_ + 3, 0);
  angular_velocity_covariance_ = (double *) malloc(9 * sizeof(double));
  std::fill(angular_velocity_covariance_, angular_velocity_covariance_ + 9, 0);
  linear_acceleration_ = (double *) malloc(3 * sizeof(double));
  std::fill(linear_acceleration_, linear_acceleration_ + 3, 0);
  linear_acceleration_covariance_ = (double *) malloc(9 * sizeof(double));
  std::fill(linear_acceleration_covariance_, linear_acceleration_covariance_ + 9, 0);

  data_ = (uint8_t *) malloc(40 * sizeof(uint8_t));
  accel_calib_data_ = (uint8_t *) malloc(28 * sizeof(uint8_t));

  // init IMU
  hardware_interface::ImuSensorHandle imu_handle(topic_,
                                                 frame_,
                                                 orientation_,
                                                 orientation_covariance_,
                                                 angular_velocity_,
                                                 angular_velocity_covariance_,
                                                 linear_acceleration_,
                                                 linear_acceleration_covariance_);
  imu_interface_.registerHandle(imu_handle);
  parent_->registerInterface(&imu_interface_);

  // make services
  imu_ranges_service_ = nh_.advertiseService("/imu/set_imu_ranges", &ImuHardwareInterface::setIMURanges, this);
  calibrate_gyro_service_ = nh_.advertiseService("/imu/calibrate_gyro", &ImuHardwareInterface::calibrateGyro, this);
  reset_gyro_calibration_service_ =
      nh_.advertiseService("/imu/reset_gyro_calibration", &ImuHardwareInterface::resetGyroCalibration, this);
  complementary_filter_params_service_ = nh_.advertiseService("/imu/set_complementary_filter_params",
                                                              &ImuHardwareInterface::setComplementaryFilterParams,
                                                              this);
  calibrate_accel_service_ = nh_.advertiseService("/imu/calibrate_accel", &ImuHardwareInterface::calibrateAccel, this);
  read_accel_calibration_service_ =
      nh_.advertiseService("/imu/read_accel_calibration", &ImuHardwareInterface::readAccelCalibration, this);
  reset_accel_calibration_service_ =
      nh_.advertiseService("/imu/reset_accel_calibration", &ImuHardwareInterface::resetAccelCalibraton, this);
  set_accel_calib_threshold_service_ = nh_.advertiseService("/imu/set_accel_calibration_threshold",
                                                            &ImuHardwareInterface::setAccelCalibrationThreshold,
                                                            this);

  diagnostic_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10, true);

  // read the current values in the IMU module so that they can later be displayed in diagnostic message
  bitbots_msgs::AccelerometerCalibrationRequest req = bitbots_msgs::AccelerometerCalibrationRequest();
  bitbots_msgs::AccelerometerCalibrationResponse resp = bitbots_msgs::AccelerometerCalibrationResponse();
  readAccelCalibration(req, resp);
  if (driver_->readMultipleRegisters(id_, 102, 14, data_)) {
    gyro_range_ = data_[0];
    accel_range_ = data_[1];
    calibrate_gyro_ = data_[2];
    reset_gyro_calibration_ = data_[3];
    do_adaptive_gain_ = data_[4];
    do_bias_estimation_ = data_[5];
    accel_gain_ = dxlMakeFloat(data_ + 6);
    bias_alpha_ = dxlMakeFloat(data_ + 10);;
  } else {
    ROS_WARN("Could not read IMU %s config values in init", name_.c_str());
  }

  return true;
}

void ImuHardwareInterface::read(const ros::Time &t, const ros::Duration &dt) {
  /**
   * Reads the IMU
   */
  bool read_successful = true;
  if (driver_->readMultipleRegisters(id_, 36, 40, data_)) {
    // sometimes we only get 0 right after power on, don't use that data
    // test on orientation is sufficient as 0,0,0,0 would not be a valid quaternion
    if (dxlMakeFloat(data_ + 24) + dxlMakeFloat(data_ + 28) + dxlMakeFloat(data_ + 32) + dxlMakeFloat(data_ + 36)
        != 0) {
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
    }
  } else {
    ROS_ERROR_THROTTLE(1.0, "Couldn't read IMU");
    read_successful = false;
  }

  // publish diagnostic messages each 100 frames
  if (diag_counter_ % 100 == 0) {
    // diagnostics. check if values are changing, otherwise there is a connection error on the board
    diagnostic_msgs::DiagnosticArray array_msg = diagnostic_msgs::DiagnosticArray();
    std::vector<diagnostic_msgs::DiagnosticStatus> array = std::vector<diagnostic_msgs::DiagnosticStatus>();
    array_msg.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status = diagnostic_msgs::DiagnosticStatus();
    // add prefix CORE to sort in diagnostic analyser
    status.name = "IMU" + name_;
    status.hardware_id = std::to_string(id_);
    std::map<std::string, std::string> map;

    map.insert(std::make_pair("Gyro Range", std::to_string(gyro_range_)));
    map.insert(std::make_pair("Accel Range", std::to_string(accel_range_)));
    map.insert(std::make_pair("Accel Gain", std::to_string(accel_gain_)));
    map.insert(std::make_pair("Bias Alpha", std::to_string(bias_alpha_)));
    map.insert(std::make_pair("Adaptive Gain", std::to_string(do_adaptive_gain_)));
    map.insert(std::make_pair("Bias Estimation", std::to_string(do_bias_estimation_)));
    map.insert(std::make_pair("Adaptive Gain", std::to_string(do_adaptive_gain_)));
    map.insert(std::make_pair("Accel Calib Thresh", std::to_string(accel_calib_threshold_read_)));
    map.insert(std::make_pair("Accel Calib Bias 0", std::to_string(accel_calib_bias_[0])));
    map.insert(std::make_pair("Accel Calib Bias 1", std::to_string(accel_calib_bias_[1])));
    map.insert(std::make_pair("Accel Calib Bias 2", std::to_string(accel_calib_bias_[2])));
    map.insert(std::make_pair("Accel Calib Scale 0", std::to_string(accel_calib_scale_[0])));
    map.insert(std::make_pair("Accel Calib Scale 1", std::to_string(accel_calib_scale_[1])));
    map.insert(std::make_pair("Accel Calib Scale 2", std::to_string(accel_calib_scale_[2])));

    if (read_successful) {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "OK";
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::STALE;
      status.message = "No response";
    }
    std::vector<diagnostic_msgs::KeyValue> keyValues = std::vector<diagnostic_msgs::KeyValue>();
    // itarate through map and save it into values
    for (auto const &ent1 : map) {
      diagnostic_msgs::KeyValue key_value = diagnostic_msgs::KeyValue();
      key_value.key = ent1.first;
      key_value.value = ent1.second;
      keyValues.push_back(key_value);
    }
    status.values = keyValues;
    array.push_back(status);
    array_msg.status = array;
    diagnostic_pub_.publish(array_msg);
  }
  diag_counter_++;
}

bool ImuHardwareInterface::setIMURanges(bitbots_msgs::IMURangesRequest &req, bitbots_msgs::IMURangesResponse &resp) {
  accel_range_ = req.accel_range;
  gyro_range_ = req.gyro_range;
  write_ranges_ = true;
  return true;
}

bool ImuHardwareInterface::calibrateGyro(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp) {
  calibrate_gyro_ = true;
  return true;
}

bool ImuHardwareInterface::resetGyroCalibration(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp) {
  reset_gyro_calibration_ = true;
  return true;
}

bool ImuHardwareInterface::setComplementaryFilterParams(bitbots_msgs::ComplementaryFilterParamsRequest &req,
                                                        bitbots_msgs::ComplementaryFilterParamsResponse &resp) {

  do_adaptive_gain_ = req.do_adaptive_gain;
  do_bias_estimation_ = req.do_bias_estimation;
  accel_gain_ = req.accel_gain;
  bias_alpha_ = req.bias_alpha;
  write_complementary_filter_params_ = true;
  return true;
}

bool ImuHardwareInterface::calibrateAccel(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp) {
  calibrate_accel_ = true;
  return true;
}

bool ImuHardwareInterface::resetAccelCalibraton(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp) {
  reset_accel_calibration_ = true;
  return true;
}

bool ImuHardwareInterface::readAccelCalibration(bitbots_msgs::AccelerometerCalibrationRequest &req,
                                                bitbots_msgs::AccelerometerCalibrationResponse &resp) {
  resp.biases.resize(3);
  resp.scales.resize(3);
  if (driver_->readMultipleRegisters(id_, 118, 28, accel_calib_data_)) {
    // save in class variables for diagnostics
    accel_calib_threshold_read_ = dxlMakeFloat(accel_calib_data_ + 0);
    accel_calib_bias_[0] = dxlMakeFloat(accel_calib_data_ + 4);
    accel_calib_bias_[1] = dxlMakeFloat(accel_calib_data_ + 8);
    accel_calib_bias_[2] = dxlMakeFloat(accel_calib_data_ + 12);
    accel_calib_scale_[0] = dxlMakeFloat(accel_calib_data_ + 16);
    accel_calib_scale_[1] = dxlMakeFloat(accel_calib_data_ + 20);
    accel_calib_scale_[2] = dxlMakeFloat(accel_calib_data_ + 24);

    resp.threshold = accel_calib_threshold_read_;
    resp.biases[0] = accel_calib_bias_[0];
    resp.biases[1] = accel_calib_bias_[1];
    resp.biases[2] = accel_calib_bias_[2];
    resp.scales[0] = accel_calib_scale_[0];
    resp.scales[1] = accel_calib_scale_[1];
    resp.scales[2] = accel_calib_scale_[2];
    return true;
  } else {
    return false;
  }
}

bool ImuHardwareInterface::setAccelCalibrationThreshold(bitbots_msgs::SetAccelerometerCalibrationThresholdRequest &req,
                                                        bitbots_msgs::SetAccelerometerCalibrationThresholdResponse &resp) {
  accel_calib_threshold_read_ = req.threshold;
  accel_calib_threshold_ = accel_calib_threshold_read_;
  set_accel_calib_threshold_ = true;
  return true;
}

void ImuHardwareInterface::write(const ros::Time &t, const ros::Duration &dt) {
  if (write_ranges_) {
    ROS_INFO_STREAM("Setting Gyroscope range to " << gyroRangeToString(gyro_range_));
    ROS_INFO_STREAM("Setting Accelerometer range to " << accelRangeToString(accel_range_));
    driver_->writeRegister(id_, "Gyro_Range", gyro_range_);
    driver_->writeRegister(id_, "Accel_Range", accel_range_);
    write_ranges_ = false;
  }
  if (calibrate_gyro_) {
    ROS_INFO("Calibrating gyroscope");
    driver_->writeRegister(id_, "Calibrate_Gyro", 1);
    calibrate_gyro_ = false;
  }
  if (reset_gyro_calibration_) {
    ROS_INFO("Resetting gyroscope Calibration");
    driver_->writeRegister(id_, "Reset_Gyro_Calibration", 1);
    reset_gyro_calibration_ = false;
  }
  if (write_complementary_filter_params_) {
    ROS_INFO("Writing Complementary Filter parameters.");
    driver_->writeRegister(id_, "Do_Adaptive_Gain", do_adaptive_gain_);
    driver_->writeRegister(id_, "Do_Bias_Estimation", do_bias_estimation_);
    driver_->writeRegister(id_, "Accel_Gain", accel_gain_);
    driver_->writeRegister(id_, "Bias_Alpha", bias_alpha_);
    write_complementary_filter_params_ = false;
  }
  if (calibrate_accel_) {
    driver_->writeRegister(id_, "Calibrate_Accel", 1);
    calibrate_accel_ = false;
  }
  if (reset_accel_calibration_) {
    driver_->writeRegister(id_, "Reset_Accel_Calibration", 1);
    reset_accel_calibration_ = false;
  }
  if (set_accel_calib_threshold_) {
    uint32_t data;
    memcpy(&data, &accel_calib_threshold_, sizeof(data));
    driver_->writeRegister(id_, "Accel_Calibration_Threshold", data);
    set_accel_calib_threshold_ = false;
  }
}

void ImuHardwareInterface::setParent(hardware_interface::RobotHW *parent) {
  parent_ = parent;
}

}


