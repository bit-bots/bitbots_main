#include <bitbots_ros_control/core_hardware_interface.hpp>

namespace bitbots_ros_control {

CoreHardwareInterface::CoreHardwareInterface(rclcpp::Node::SharedPtr nh, std::shared_ptr<DynamixelDriver> &driver,
                                             int id, int read_rate) {
  nh_ = nh;
  driver_ = driver;
  id_ = id;
  read_rate_ = read_rate;
  read_counter_ = 0;
  requested_power_status_ = true;
  power_switch_status_.data = false;
  power_control_status_.data = false;
  last_read_successful_ = false;
}

bool CoreHardwareInterface::switch_power(std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                         std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
  requested_power_status_ = req->data;
  // wait for main loop to set value
  resp->success = true;
  return true;
}

bool CoreHardwareInterface::init() {
  VBAT_individual_.data.resize(6);
  power_pub_ = nh_->create_publisher<std_msgs::msg::Bool>("/core/power_switch_status", 1);
  vcc_pub_ = nh_->create_publisher<std_msgs::msg::Float64>("/core/vcc", 1);
  vbat_pub_ = nh_->create_publisher<std_msgs::msg::Float64>("/core/vbat", 1);
  vbat_individual_pub_ = nh_->create_publisher<std_msgs::msg::Float64MultiArray>("/core/vbat_cells", 1);
  vext_pub_ = nh_->create_publisher<std_msgs::msg::Float64>("/core/vext", 1);
  vdxl_pub_ = nh_->create_publisher<std_msgs::msg::Float64>("/core/vdxl", 1);
  current_pub_ = nh_->create_publisher<std_msgs::msg::Float64>("/core/current", 1);

  diagnostic_pub_ = nh_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  // service to switch power
  power_switch_service_ = nh_->create_service<std_srvs::srv::SetBool>(
      "/core/switch_power",
      std::bind(&CoreHardwareInterface::switch_power, this, std::placeholders::_1, std::placeholders::_2));
  return true;
}

bool CoreHardwareInterface::get_power_status() {
  // this is only true when the physical switch and the soft status are true
  return power_control_status_.data && power_switch_status_.data;
}

void CoreHardwareInterface::read(const rclcpp::Time &t, const rclcpp::Duration &dt) {
  /**
   * Reads the CORE board
   */

  if (read_counter_ % read_rate_ == 0) {
    read_counter_ = 0;
    // read core
    last_read_successful_ = true;
    if (driver_->readMultipleRegisters(id_, 23, 27, data_.data())) {
      // we read one string of bytes. see CORE firmware for definition of registers
      // convert to volt
      power_control_status_.data = data_[0];
      VEXT_.data = (((float)dxlMakeword(data_[5], data_[6])) * (3.3 / 1024)) * 6;
      VCC_.data = (((float)dxlMakeword(data_[7], data_[8])) * (3.3 / 1024)) * 6;
      VDXL_.data = (((float)dxlMakeword(data_[9], data_[10])) * (3.3 / 1024)) * 6;
      // convert to ampere. first go to voltage by 1024*3.3. shift by 2.5 and mulitply by volt/ampere
      current_.data = ((((float)dxlMakeword(data_[11], data_[12])) * (3.3 / 1024)) - 2.5) / -0.066;
      // we need to apply a threshold on this to see if power is on or off
      power_switch_status_.data = dxlMakeword(data_[13], data_[14]);
      // calculate cell voltages as voltages read * voltage divider ratio - previous cell voltage sum
      VBAT_individual_.data[0] = ((float)dxlMakeword(data_[15], data_[16])) * (3.3 / 1024) * (3.3 / (1.2 + 3.3));
      VBAT_individual_.data[1] =
          ((float)dxlMakeword(data_[17], data_[18])) * (3.3 / 1024) * (3.6 / (6.2 + 3.6)) - VBAT_individual_.data[0];
      VBAT_individual_.data[2] =
          ((float)dxlMakeword(data_[19], data_[20])) * (3.3 / 1024) * (2.2 / (6.8 + 2.2)) - VBAT_individual_.data[1];
      VBAT_individual_.data[3] =
          ((float)dxlMakeword(data_[21], data_[22])) * (3.3 / 1024) * (3.6 / (16.0 + 3.6)) - VBAT_individual_.data[2];
      VBAT_individual_.data[4] =
          ((float)dxlMakeword(data_[23], data_[24])) * (3.3 / 1024) * (6.2 / (36.0 + 6.2)) - VBAT_individual_.data[3];
      VBAT_individual_.data[5] =
          ((float)dxlMakeword(data_[25], data_[26])) * (3.3 / 1024) * (1.8 / (13.0 + 1.8)) - VBAT_individual_.data[4];
      VBAT_.data = ((float)dxlMakeword(data_[25], data_[26])) * (3.3 / 1024) * (1.8 / (13.0 + 1.8));

      power_pub_->publish(power_switch_status_);
      vcc_pub_->publish(VCC_);
      vbat_pub_->publish(VBAT_);
      vbat_individual_pub_->publish(VBAT_individual_);
      vext_pub_->publish(VEXT_);
      vdxl_pub_->publish(VDXL_);
      current_pub_->publish(current_);
    } else {
      RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3000, "Could not read CORE sensor");
      last_read_successful_ = false;
    }

    // diagnostics. check if values are changing, otherwise there is a connection error on the board
    diagnostic_msgs::msg::DiagnosticArray array_msg = diagnostic_msgs::msg::DiagnosticArray();
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> array = std::vector<diagnostic_msgs::msg::DiagnosticStatus>();
    array_msg.header.stamp = nh_->get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status = diagnostic_msgs::msg::DiagnosticStatus();
    // add prefix CORE to sort in diagnostic analyser
    status.name = "CORECORE";
    status.hardware_id = std::to_string(id_);
    std::map<std::string, std::string> map;

    if (power_switch_status_.data) {
      map.insert(std::make_pair("power_switch_status", "ON"));
    } else {
      map.insert(std::make_pair("power_switch_status", "OFF"));
    }
    if (power_control_status_.data) {
      map.insert(std::make_pair("power_control_status", "ON"));
    } else {
      map.insert(std::make_pair("power_control_status", "OFF"));
    }
    map.insert(std::make_pair("VCC", std::to_string(VCC_.data)));
    map.insert(std::make_pair("VBAT", std::to_string(VBAT_.data)));
    map.insert(std::make_pair("VEXT", std::to_string(VEXT_.data)));
    map.insert(std::make_pair("VDXL", std::to_string(VDXL_.data)));
    map.insert(std::make_pair("Current", std::to_string(current_.data)));

    if (last_read_successful_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "OK";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      status.message = "No response";
    }
    std::vector<diagnostic_msgs::msg::KeyValue> keyValues = std::vector<diagnostic_msgs::msg::KeyValue>();
    // iterate through map and save it into values
    for (auto const &ent1 : map) {
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
  read_counter_++;
}

void CoreHardwareInterface::write(const rclcpp::Time &t, const rclcpp::Duration &dt) {
  // we only need to write something if requested power status and current power status do not match
  // we don't use power_switch_status here because we cannot overwrite the physical switch
  if (requested_power_status_ != power_control_status_.data && last_read_successful_) {
    driver_->writeRegister(id_, "Power", requested_power_status_);
  }
}
}  // namespace bitbots_ros_control
