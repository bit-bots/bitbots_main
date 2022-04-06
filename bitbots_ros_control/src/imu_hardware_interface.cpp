#include <bitbots_ros_control/imu_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

#define gravity 9.80665

namespace bitbots_ros_control {

ImuHardwareInterface::ImuHardwareInterface(rclcpp::Node::SharedPtr nh,
                                           std::shared_ptr<DynamixelDriver> &driver,
                                           int id,
                                           std::string topic,
                                           std::string frame,
                                           std::string name) {
  nh_ = nh;
  driver_ = driver;
  id_ = id;
  topic_ = topic;
  frame_ = frame;
  name_ = name;
  diag_counter_ = 0;
}

bool ImuHardwareInterface::init() {
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

  // make services
  imu_ranges_service_ = nh_->create_service<bitbots_msgs::srv::IMURanges>(
      "/imu/set_imu_ranges", &ImuHardwareInterface::setIMURanges);
  calibrate_gyro_service_ = nh_->create_service<std_srvs::srv::Empty>(
      "/imu/calibrate_gyro", &ImuHardwareInterface::calibrateGyro);
  reset_gyro_calibration_service_ = nh_->create_service<std_srvs::srv::Empty>(
      "/imu/reset_gyro_calibration", &ImuHardwareInterface::resetGyroCalibration);
  complementary_filter_params_service_ = nh_->create_service<bitbots_msgs::srv::ComplementaryFilterParams>(
      "/imu/set_complementary_filter_params", &ImuHardwareInterface::setComplementaryFilterParams);
  calibrate_accel_service_ = nh_->create_service<std_srvs::srv::Empty>(
      "/imu/calibrate_accel", &ImuHardwareInterface::calibrateAccel);
  read_accel_calibration_service_ = nh_->create_service<bitbots_msgs::srv::AccelerometerCalibration>(
      "/imu/read_accel_calibration", &ImuHardwareInterface::readAccelCalibration);
  reset_accel_calibration_service_ = nh_->create_service<std_srvs::srv::Empty>(
      "/imu/reset_accel_calibration", &ImuHardwareInterface::resetAccelCalibraton);
  set_accel_calib_threshold_service_ = nh_->create_service<bitbots_msgs::srv::SetAccelerometerCalibrationThreshold>(
      "/imu/set_accel_calibration_threshold", &ImuHardwareInterface::setAccelCalibrationThreshold);

  diagnostic_pub_ = nh_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  // read the current values in the IMU module so that they can later be displayed in diagnostic message
  bitbots_msgs::srv::AccelerometerCalibration::Request req = bitbots_msgs::srv::AccelerometerCalibration::Request();
  bitbots_msgs::srv::AccelerometerCalibration::Response resp = bitbots_msgs::srv::AccelerometerCalibration::Response();
  readAccelCalibration(req, resp);
  if (driver_->readMultipleRegisters(id_, 102, 16, data_)) {
    gyro_range_ = data_[0];
    accel_range_ = data_[1];
    calibrate_gyro_ = data_[2];
    reset_gyro_calibration_ = data_[3];
    do_adaptive_gain_ = data_[6];
    do_bias_estimation_ = data_[7];
    accel_gain_ = dxlMakeFloat(data_ + 8);
    bias_alpha_ = dxlMakeFloat(data_ + 12);
  } else {
    RCLCPP_WARN(nh_->get_logger(), "Could not read IMU %s config values in init", name_.c_str());
  }

  //set filter values differently if specified in the config and write them
  nh_->declare_parameter<bool>("imu/do_adaptive_gain", do_adaptive_gain_);
  do_adaptive_gain_ = nh_->get_parameter("imu/do_adaptive_gain").as_bool();
  nh_->declare_parameter<bool>("imu/do_bias_estimation", do_bias_estimation_);
  do_bias_estimation_ = nh_->get_parameter("imu/do_bias_estimation").as_bool();
  nh_->declare_parameter<double>("imu/accel_gain", accel_gain_);
  accel_gain_ = nh_->get_parameter("imu/accel_gain").as_double();
  nh_->declare_parameter<double>("imu/bias_alpha", bias_alpha_);
  bias_alpha_ = nh_->get_parameter("imu/bias_alpha").as_double();

  write_complementary_filter_params_ = true;
  write(rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(1e9 * 0));

  return true;
}

void ImuHardwareInterface::read(const rclcpp::Time &t, const rclcpp::Duration &dt) {
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
    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1.0, "Couldn't read IMU");
    read_successful = false;
  }

  // publish diagnostic messages each 100 frames
  if (diag_counter_ % 100 == 0) {
    // diagnostics. check if values are changing, otherwise there is a connection error on the board
    diagnostic_msgs::msg::DiagnosticArray array_msg = diagnostic_msgs::msg::DiagnosticArray();
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> array = std::vector<diagnostic_msgs::msg::DiagnosticStatus>();
    array_msg.header.stamp = nh_->get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status = diagnostic_msgs::msg::DiagnosticStatus();
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
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "OK";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      status.message = "No response";
    }
    std::vector<diagnostic_msgs::msg::KeyValue> keyValues = std::vector<diagnostic_msgs::msg::KeyValue>();
    // itarate through map and save it into values
    for (auto const &ent1: map) {
      diagnostic_msgs::msg::KeyValue key_value = diagnostic_msgs::msg::KeyValue();
      key_value.key = ent1.first;
      key_value.value = ent1.second;
      keyValues.push_back(key_value);
    }
    status.values = keyValues;
    array.push_back(status);
    array_msg.status = array;
    diagnostic_pub_->publish(array_msg);
  }
  diag_counter_++;
}

bool ImuHardwareInterface::setIMURanges(bitbots_msgs::srv::IMURanges::Request &req,
                                        bitbots_msgs::srv::IMURanges::Response &resp) {
  accel_range_ = req.accel_range;
  gyro_range_ = req.gyro_range;
  write_ranges_ = true;
  return true;
}

bool ImuHardwareInterface::calibrateGyro(std_srvs::srv::Empty::Request &req, std_srvs::srv::Empty::Response &resp) {
  calibrate_gyro_ = true;
  return true;
}

bool ImuHardwareInterface::resetGyroCalibration(std_srvs::srv::Empty::Request &req,
                                                std_srvs::srv::Empty::Response &resp) {
  reset_gyro_calibration_ = true;
  return true;
}

bool ImuHardwareInterface::setComplementaryFilterParams(bitbots_msgs::srv::ComplementaryFilterParams::Request &req,
                                                        bitbots_msgs::srv::ComplementaryFilterParams::Response &resp) {

  do_adaptive_gain_ = req.do_adaptive_gain;
  do_bias_estimation_ = req.do_bias_estimation;
  accel_gain_ = req.accel_gain;
  bias_alpha_ = req.bias_alpha;
  write_complementary_filter_params_ = true;
  return true;
}

bool ImuHardwareInterface::calibrateAccel(std_srvs::srv::Empty::Request &req, std_srvs::srv::Empty::Response &resp) {
  calibrate_accel_ = true;
  return true;
}

bool ImuHardwareInterface::resetAccelCalibraton(std_srvs::srv::Empty::Request &req,
                                                std_srvs::srv::Empty::Response &resp) {
  reset_accel_calibration_ = true;
  return true;
}

bool ImuHardwareInterface::readAccelCalibration(bitbots_msgs::srv::AccelerometerCalibration::Request &req,
                                                bitbots_msgs::srv::AccelerometerCalibration::Response &resp) {
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

bool ImuHardwareInterface::setAccelCalibrationThreshold(bitbots_msgs::srv::SetAccelerometerCalibrationThreshold::Request &req,
                                                        bitbots_msgs::srv::SetAccelerometerCalibrationThreshold::Response &resp) {
  accel_calib_threshold_read_ = req.threshold;
  accel_calib_threshold_ = accel_calib_threshold_read_;
  set_accel_calib_threshold_ = true;
  return true;
}

void ImuHardwareInterface::write(const rclcpp::Time &t, const rclcpp::Duration &dt) {
  if (write_ranges_) {
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Setting Gyroscope range to " << gyroRangeToString(gyro_range_));
    RCLCPP_INFO_STREAM(nh_->get_logger(), "Setting Accelerometer range to " << accelRangeToString(accel_range_));
    driver_->writeRegister(id_, "Gyro_Range", gyro_range_);
    driver_->writeRegister(id_, "Accel_Range", accel_range_);
    write_ranges_ = false;
  }
  if (calibrate_gyro_) {
    RCLCPP_INFO(nh_->get_logger(), "Calibrating gyroscope");
    driver_->writeRegister(id_, "Calibrate_Gyro", 1);
    calibrate_gyro_ = false;
  }
  if (reset_gyro_calibration_) {
    RCLCPP_INFO(nh_->get_logger(), "Resetting gyroscope Calibration");
    driver_->writeRegister(id_, "Reset_Gyro_Calibration", 1);
    reset_gyro_calibration_ = false;
  }
  if (write_complementary_filter_params_) {
    RCLCPP_INFO(nh_->get_logger(), "Writing Complementary Filter parameters.");
    driver_->writeRegister(id_, "Do_Adaptive_Gain", do_adaptive_gain_);
    driver_->writeRegister(id_, "Do_Bias_Estimation", do_bias_estimation_);

    // dynamixel library requires data (up to 4 bytes) as uint32_t
    // float would be cast to int losing the decimal and being interpreted wrongly on the device
    uint32_t data;
    memcpy(&data, &accel_gain_, sizeof(data));
    driver_->writeRegister(id_, "Accel_Gain", data);
    memcpy(&data, &bias_alpha_, sizeof(data));
    driver_->writeRegister(id_, "Bias_Alpha", data);

    write_complementary_filter_params_ = false;
  }
  if (calibrate_accel_) {
    RCLCPP_ERROR(nh_->get_logger(), "Disabled for normal users for safety reasons, uncomment code to use");
    //driver_->writeRegister(id_, "Calibrate_Accel", 1);
    calibrate_accel_ = false;
  }
  if (reset_accel_calibration_) {
    RCLCPP_ERROR(nh_->get_logger(), "Disabled for normal users for safety reasons, uncomment code to use");
    //driver_->writeRegister(id_, "Reset_Accel_Calibration", 1);
    reset_accel_calibration_ = false;
  }
  if (set_accel_calib_threshold_) {
    uint32_t data;
    memcpy(&data, &accel_calib_threshold_, sizeof(data));
    driver_->writeRegister(id_, "Accel_Calibration_Threshold", data);
    set_accel_calib_threshold_ = false;
  }
}
}


