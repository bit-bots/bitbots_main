#include <bitbots_ros_control/wolfgang_hardware_interface.h>

namespace bitbots_ros_control {

/**
 * This class provides a combination of multiple hardware interfaces to construct a complete Wolfgang robot.
 * It is similar to a CombinedRobotHw class as specified in ros_control, but it is changed a bit to make sharing of
 * a common bus driver over multiple hardware interfaces possible.
 */
WolfgangHardwareInterface::WolfgangHardwareInterface(rclcpp::Node::SharedPtr nh) : servo_interface_(nh) {
  nh_ = nh;
  first_ping_error_ = true;
  core_present_ = false;
  last_power_status_ = false;
  current_power_status_ = false;
  speak_pub_ = nh->create_publisher<humanoid_league_msgs::msg::Audio>("/speak", 1);

  // load parameters
  nh_->get_parameter("only_imu", only_imu_);
  nh_->get_parameter("only_pressure", only_pressure_);
  if (only_imu_) RCLCPP_WARN(nh_->get_logger(), "Starting in only IMU mode");
  if (only_pressure_) RCLCPP_WARN(nh_->get_logger(), "starting in only pressure sensor mode");

  if (only_pressure_ && only_imu_) {
    RCLCPP_ERROR(nh_->get_logger(), "only_imu AND only_pressure was set to true");
    exit(1);
  }

  // get list of all bus devices
  rcl_interfaces::msg::ListParametersResult device_name_list = nh_->list_parameters({"device_info"}, 3);

  // Convert dxls to native type: a vector of tuples with name and id for sorting purposes
  std::vector<std::pair<std::string, int>> dxl_devices;
  for (const std::string &parameter_name: device_name_list.names) {
    // we get directly the parameters and not the groups. use id parameter to identify them
    if (parameter_name.find(".id")!=std::string::npos) {
      int id = nh->get_parameter(parameter_name).as_int();
      // remove "device_info." and ".id"
      std::string device_name = parameter_name.substr(12, parameter_name.size() - 3 - 12);
      dxl_devices.emplace_back(device_name, id);
    }
  }

  // sort the devices by id. This way the devices will always be read and written in ID order later, making debug easier.
  std::sort(dxl_devices.begin(), dxl_devices.end(),
            [](std::pair<std::string, int> &a, std::pair<std::string, int> &b) { return a.second < b.second; });

  // create overall servo interface since we need a single interface for the controllers
  servo_interface_ = DynamixelServoHardwareInterface(nh);

  // try to ping all devices on the list, add them to the driver and create corresponding hardware interfaces
  // try until interruption to enable the user to turn on the power
  while (rclcpp::ok()) {
    if (create_interfaces(dxl_devices)) {
      break;
    }
    nh_->get_clock()->sleep_for(rclcpp::Duration::from_seconds(3));
  }
}

bool WolfgangHardwareInterface::create_interfaces(std::vector<std::pair<std::string, int>> dxl_devices) {
  interfaces_ = std::vector<std::vector<bitbots_ros_control::HardwareInterface * >>();
  // init bus drivers
  std::vector<std::string> pinged;
  rcl_interfaces::msg::ListParametersResult port_list = nh_->list_parameters({"port_info"}, 3);
  for (const std::string &parameter_name: port_list.names) {
    // we get directly the parameters and not the groups. use id parameter to identify them
    if (parameter_name.find(".device_file")!=std::string::npos) {
      std::string port_name = parameter_name.substr(10, parameter_name.size() - 11 - 11);
      // read bus driver specifications from config
      std::string device_file = nh_->get_parameter("port_info." + port_name + ".device_file").as_string();
      int baudrate = nh_->get_parameter("port_info." + port_name + ".baudrate").as_int();
      int protocol_version = nh_->get_parameter("port_info." + port_name + ".protocol_version").as_int();
      auto driver = std::make_shared<DynamixelDriver>();
      if (!driver->init(device_file.c_str(), uint32_t(baudrate))) {
        RCLCPP_ERROR(nh_->get_logger(), "Error opening serial port %s", device_file.c_str());
        speakError(speak_pub_, "Error opening serial port");
        sleep(1);
        exit(1);
      }
      // some interface seem to produce some gitter directly after connecting. wait or it will interfere with pings
      // uncomment the following line if you are using such an interface
      // sleep(1);
      driver->setPacketHandler(protocol_version);
      std::vector<bitbots_ros_control::HardwareInterface *> interfaces_on_port;
      // iterate over all devices and ping them to see what is connected to this bus
      std::vector<std::tuple<int, std::string, float, float>> servos_on_port;
      for (std::pair<std::string, int> &device: dxl_devices) {
        //RCLCPP_INFO_STREAM(nh_->get_logger(), device.first);
        std::string name = device.first;
        int id = device.second;
        if (std::find(pinged.begin(), pinged.end(), device.first)!=pinged.end()) {
          // we already found this and don't have to search again
        } else {
          int model_number_specified;
          nh_->get_parameter("device_info." + name + ".model_number", model_number_specified);
          // some devices provide more than one type of interface, e.g. the IMU provides additionally buttons and LEDs
          std::string interface_type;
          nh_->get_parameter("device_info." + name + ".interface_type", interface_type);
          uint16_t model_number_specified_16 = uint16_t(model_number_specified);
          uint16_t *model_number_returned_16 = new uint16_t;
          if (driver->ping(uint8_t(id), model_number_returned_16)) {
            // check if the specified model number matches the actual model number of the device
            if (model_number_specified_16!=*model_number_returned_16) {
              RCLCPP_WARN(nh_->get_logger(), "Model number of id %d does not match", id);
            }
            // ping was successful, add device correspondingly
            // only add them if the mode is set correspondingly
            // TODO maybe move more of the parameter stuff in the init of the modules instead of doing everything here
            if (model_number_specified==0xABBA && interface_type=="CORE") {
              // CORE
              int read_rate;
              nh_->get_parameter("device_info." + name + ".read_rate", read_rate);
              driver->setTools(model_number_specified_16, id);
              core_interface_ = new CoreHardwareInterface(nh_, driver, id, read_rate);
              // turn on power, just to be sure
              core_interface_->write(nh_->get_clock()->now(), rclcpp::Duration::from_nanoseconds(1e9*0));
              interfaces_on_port.push_back(core_interface_);
              core_present_ = true;
            } else if (model_number_specified == 0 && !only_imu_) {  // model number is currently 0 on foot sensors
              // bitfoot
              std::string topic;
              if (!nh_->get_parameter("device_info." + name + ".topic", topic)) {
                RCLCPP_WARN(nh_->get_logger(), "Bitfoot topic not specified");
              }
              BitFootHardwareInterface *interface = new BitFootHardwareInterface(nh_, driver, id, topic, name);
              interfaces_on_port.push_back(interface);
            } else if (model_number_specified == 0xBAFF && interface_type == "IMU" && !only_pressure_) {
              //IMU
              std::string topic;
              if (!nh_->get_parameter("device_info." + name + ".topic", topic)) {
                RCLCPP_WARN(nh_->get_logger(), "IMU topic not specified");
              }
              std::string frame;
              if (!nh_->get_parameter("device_info." + name + ".frame", frame)) {
                RCLCPP_WARN(nh_->get_logger(), "IMU frame not specified");
              }
              driver->setTools(model_number_specified_16, id);
              ImuHardwareInterface *interface = new ImuHardwareInterface(nh_, driver, id, topic, frame, name);
              /* Hardware interfaces must be registered at the main RobotHW class.
               * Therefore, a pointer to this class is passed down to the RobotHW classes
               * registering further interfaces */
              interfaces_on_port.push_back(interface);
            } else if (model_number_specified == 0xBAFF && interface_type == "Button" && !only_pressure_) {
              // Buttons
              std::string topic;
              if (!nh_->get_parameter("device_info." + name + ".topic", topic)) {
                RCLCPP_WARN(nh_->get_logger(), "Button topic not specified");
              }
              int read_rate;
              nh_->get_parameter("device_info." + name + ".read_rate", read_rate);
              interfaces_on_port.push_back(new ButtonHardwareInterface(nh_, driver, id, topic, read_rate));
            } else if ((model_number_specified == 0xBAFF || model_number_specified == 0xABBA) &&
                       interface_type == "LED" && !only_pressure_) {
              // LEDs
              int number_of_LEDs, start_number;
              nh_->get_parameter("device_info." + name + ".number_of_LEDs", number_of_LEDs);
              nh_->get_parameter("device_info." + name + ".start_number", start_number);
              interfaces_on_port.push_back(new LedsHardwareInterface(nh_, driver, id, number_of_LEDs, start_number));
            } else if ((model_number_specified == 311 || model_number_specified == 321 ||
                        model_number_specified == 1100) &&
                       !only_pressure_ && !only_imu_) {
              // Servos
              // We need to add the tool to the driver for later reading and writing
              driver->setTools(model_number_specified_16, id);
              float mounting_offset;
              nh_->get_parameter_or("device_info." + name + ".mounting_offset", mounting_offset, 0.0f);
              float joint_offset;
              nh_->get_parameter_or("device_info." + name + ".joint_offset", joint_offset, 0.0f);
              servos_on_port.push_back(std::make_tuple(id, name, mounting_offset, joint_offset));
            } else {
              if (!only_pressure_ && !only_imu_) {
                RCLCPP_WARN(nh_->get_logger(), "Could not identify device for ID %d", id);
              }
            }
            pinged.push_back(name);
          }
          delete model_number_returned_16;
        }
      }
      // create a servo bus interface if there were servos found on this bus
      if (servos_on_port.size() > 0) {
        ServoBusInterface *interface = new ServoBusInterface(nh_, driver, servos_on_port);
        interfaces_on_port.push_back(interface);
        servo_interface_.addBusInterface(interface);
      }
      // add vector of interfaces on this port to overall collection of interfaces
      interfaces_.push_back(interfaces_on_port);
    }
  }

  if (pinged.size()!=dxl_devices.size()) {
    // when we only have 1 or two devices it's only the core
    if (pinged.empty() || pinged.size()==1 || pinged.size()==2) {
      RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 5000, "Could not start ros control. Power is off!");
      speakError(speak_pub_, "Could not start ross control. Power is off!");
    } else {
      if (first_ping_error_) {
        first_ping_error_ = false;
      } else {
        RCLCPP_ERROR(nh_->get_logger(), "Could not ping all devices!");
        speakError(speak_pub_, "error starting ross control");
        // check which devices were not pinged successful
        for (std::pair<std::string, int> &device: dxl_devices) {
          if (std::find(pinged.begin(), pinged.end(), device.first)!=pinged.end()) {
          } else {
            RCLCPP_ERROR(nh_->get_logger(), "%s with id %d missing", device.first.c_str(), device.second);
          }
        }
      }
    }
    return false;
  } else {
    speakError(speak_pub_, "ross control startup successful");
    return true;
  }
}

void threaded_init(std::vector<HardwareInterface *> &port_interfaces, rclcpp::Node::SharedPtr &nh, int &success) {
  success = true;
  for (HardwareInterface *interface: port_interfaces) {
    success &= interface->init();
  }
}

bool WolfgangHardwareInterface::init() {
  // iterate through all ports
  std::vector<std::thread> threads;
  std::vector<int *> successes;
  int i = 0;
  for (std::vector<HardwareInterface *> &port_interfaces: interfaces_) {
    // iterate through all interfaces on this port
    // we use an int instead of bool, since std::ref can't handle bool
    int suc = 0;
    successes.push_back(&suc);
    threads.push_back(std::thread(threaded_init, std::ref(port_interfaces), std::ref(nh_), std::ref(suc)));
    i++;
  }
  // wait for all inits to finish
  for (std::thread &thread: threads) {
    thread.join();
  }
  // see if all inits were successful
  bool success = true;
  for (bool s: successes) {
    success &= s;
  }
  // init servo interface last after all servo busses are there
  success &= servo_interface_.init();
  return success;
}

void threaded_read(std::vector<HardwareInterface *> &port_interfaces,
                   const rclcpp::Time &t,
                   const rclcpp::Duration &dt) {
  for (HardwareInterface *interface: port_interfaces) {
    interface->read(t, dt);
  }
}

void WolfgangHardwareInterface::read(const rclcpp::Time &t, const rclcpp::Duration &dt) {
  // give feedback to power changes
  if (core_present_){
    if(current_power_status_ && !last_power_status_){
      speakError(speak_pub_, "Power switched on!");
    }else if(!current_power_status_ && last_power_status_){
      speakError(speak_pub_, "Power switched off!");
    }
  }
  if (!core_present_ || current_power_status_){
    // only read all hardware if power is on
    std::vector<std::thread> threads;
    // start all reads
    for (std::vector<HardwareInterface *> &port_interfaces: interfaces_) {
      threads.push_back(std::thread(threaded_read, std::ref(port_interfaces), std::ref(t), std::ref(dt)));
    }
    // wait for all reads to finish
    for (std::thread &thread: threads) {
      thread.join();
    }
    // aggregate all servo values for controller
    servo_interface_.read(t, dt);
    if (core_present_){
      last_power_status_ = current_power_status_;
      current_power_status_ = core_interface_->get_power_status();
    }
  }else{
    // read core to see if power is back on
    core_interface_->read(t, dt);
    last_power_status_ = current_power_status_;
    current_power_status_ = core_interface_->get_power_status();
    servo_interface_.read(t, dt);
  }
}

void threaded_write(std::vector<HardwareInterface *> &port_interfaces,
                    const rclcpp::Time &t,
                    const rclcpp::Duration &dt) {
  for (HardwareInterface *interface: port_interfaces) {
    interface->write(t, dt);
  }
}

void WolfgangHardwareInterface::write(const rclcpp::Time &t, const rclcpp::Duration &dt) {
  if (core_present_ && !last_power_status_ && current_power_status_ &&
      nh_->get_parameter("servos.set_ROM_RAM").as_bool()) {
    // when we can read the power and see that it was just switched on, we write the ROM RAM again
    servo_interface_.writeROMRAM(false);
  } else if(core_present_ && !current_power_status_){
    // if power is off only write CORE
    core_interface_->write(t, dt);
  } else {
    // write all controller values to interfaces
    servo_interface_.write(t, dt);
    std::vector<std::thread> threads;
    // start all writes
    for (std::vector<HardwareInterface *> &port_interfaces: interfaces_) {
      threads.push_back(std::thread(threaded_write, std::ref(port_interfaces), std::ref(t), std::ref(dt)));
    }

    // wait for all writes to finish
    for (std::thread &thread: threads) {
      thread.join();
    }
  }
}
}