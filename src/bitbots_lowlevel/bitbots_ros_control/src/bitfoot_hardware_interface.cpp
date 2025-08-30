#include <bitbots_ros_control/bitfoot_hardware_interface.hpp>
#include <bitbots_ros_control/utils.hpp>

namespace bitbots_ros_control {

BitFootHardwareInterface::BitFootHardwareInterface(rclcpp::Node::SharedPtr nh, std::shared_ptr<DynamixelDriver> &driver,
                                                   int id, std::string topic_name, std::string name) {
  nh_ = nh;
  driver_ = driver;
  id_ = id;
  topic_name_ = topic_name;
  name_ = name;
}

bool BitFootHardwareInterface::init() {
  current_pressure_.resize(4, std::vector<double>());
  pressure_pub_ = nh_->create_publisher<bitbots_msgs::msg::FootPressure>(topic_name_, 1);
  diagnostic_pub_ = nh_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  return true;
}

void BitFootHardwareInterface::read(const rclcpp::Time &t, const rclcpp::Duration &dt) {
  /**
   * Reads the foot pressure sensors of the BitFoot
   */

  // read foot
  bool read_successful = true;
  if (driver_->readMultipleRegisters(id_, 36, 16, data_.data())) {
    for (int i = 0; i < 4; i++) {
      float pres = dxlMakeFloat(&data_[i * 4]);
      // we directly provide raw data since the scaling has to be calibrated by another node for every robot anyway
      current_pressure_[i].push_back((double)pres);
    }
  } else {
    RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 3000, "Could not read %s", name_.c_str());
    read_successful = false;
  }

  msg_.header.stamp = nh_->get_clock()->now();
  msg_.left_front = current_pressure_[0].back();
  msg_.right_front = current_pressure_[1].back();
  msg_.left_back = current_pressure_[2].back();
  msg_.right_back = current_pressure_[3].back();
  pressure_pub_->publish(msg_);

  // wait till we have 10 values
  if (current_pressure_[0].size() > 10) {
    // erase older value
    current_pressure_[0].erase(current_pressure_[0].begin());
    // diagnostics. check if values are changing, otherwise there is a connection error on the board
    diagnostic_msgs::msg::DiagnosticArray array_msg = diagnostic_msgs::msg::DiagnosticArray();
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> array = std::vector<diagnostic_msgs::msg::DiagnosticStatus>();
    array_msg.header.stamp = nh_->get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status = diagnostic_msgs::msg::DiagnosticStatus();
    // add prefix PS for pressure sensor to sort in diagnostic analyser
    status.name = "PS" + name_;
    status.hardware_id = std::to_string(id_);
    std::map<std::string, std::string> map;
    bool all_okay = true;
    for (int i = 0; i < 4; i++) {
      std::string gauge_name;
      if (i == 0) {
        gauge_name = "Strain Gauge Left Front";
      } else if (i == 1) {
        gauge_name = "Strain Gauge Right Front";
      } else if (i == 2) {
        gauge_name = "Strain Gauge Left Back";
      } else if (i == 3) {
        gauge_name = "Strain Gauge Right Back";
      }

      bool okay = false;
      double last = 0;
      for (size_t j = 0; j < current_pressure_[i].size(); j++) {
        if (last != current_pressure_[0][j]) {
          okay = true;
          break;
        }
      }
      all_okay &= okay;
      std::string okay_string;
      if (okay) {
        okay_string = "Okay";
      } else {
        okay_string = "Error";
      }
      map.insert(std::make_pair(gauge_name, okay_string));
    }
    if (read_successful) {
      if (all_okay) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "OK";
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Cable problem to strain gauge";
      }
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Could not read foot sensor";
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
}

// we dont write anything to the pressure sensors
void BitFootHardwareInterface::write(const rclcpp::Time &t, const rclcpp::Duration &dt) {}
}  // namespace bitbots_ros_control
