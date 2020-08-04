#include <bitbots_ros_control/wolfgang_hardware_interface.h>
#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control {

/**
 * This class provides a combination of multiple hardware interfaces to construct a complete Wolfgang robot.
 * It is similar to a CombinedRobotHw class as specified in ros_control, but it is changed a bit to make sharing of
 * a common bus driver over multiple hardware interfaces possible.
 */
WolfgangHardwareInterface::WolfgangHardwareInterface(ros::NodeHandle &nh) {
  speak_pub_ = nh.advertise<humanoid_league_msgs::Speak>("/speak", 1);

  // load parameters
  ROS_INFO_STREAM("Loading parameters from namespace " << nh.getNamespace());
  nh.param<bool>("only_imu", only_imu_, false);
  if (only_imu_) ROS_WARN("Starting in only IMU mode");
  nh.param<bool>("only_pressure", only_pressure_, false);
  if (only_pressure_) ROS_WARN("starting in only pressure sensor mode");

  if (only_pressure_ && only_imu_) {
    ROS_ERROR("only_imu AND only_pressure was set to true");
    exit(1);
  }

  // get list of all bus devices
  XmlRpc::XmlRpcValue dxls_xml;
  nh.getParam("device_info", dxls_xml);
  ROS_ASSERT(dxls_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  // Convert dxls to native type: a vector of tuples with name and id for sorting purposes
  std::vector<std::pair<std::string, int>> dxl_devices;
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = dxls_xml.begin(); it != dxls_xml.end(); ++it) {
    std::string name = it->first;
    XmlRpc::XmlRpcValue data = it->second;
    int id = data["id"];
    dxl_devices.emplace_back(name, id);
  }

  // sort the devices by id. This way the devices will always be read and written in ID order later, making debug easier.
  std::sort(dxl_devices.begin(), dxl_devices.end(),
            [](std::pair<std::string, int> &a, std::pair<std::string, int> &b) { return a.second < b.second; });

  // create overall servo interface since we need a single interface for the controllers
  servo_interface_ = DynamixelServoHardwareInterface();
  servo_interface_.setParent(this);
  /* duplicate?
  // set the dynamic reconfigure and load standard params for servo interface
 dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig> server;
 dynamic_reconfigure::Server<bitbots_ros_control::dynamixel_servo_hardware_interface_paramsConfig>::CallbackType
     f;
 f = boost::bind(&bitbots_ros_control::DynamixelServoHardwareInterface::reconfCallback, servo_interface_, _1, _2);
 server.setCallback(f);*/

  // init bus drivers
  std::vector<std::string> pinged;
  XmlRpc::XmlRpcValue port_xml;
  nh.getParam("port_info", port_xml);
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = port_xml.begin(); it != port_xml.end(); ++it) {
    // read bus driver specifications from config
    XmlRpc::XmlRpcValue port_data = it->second;
    std::string device_file = port_data["device_file"];
    int baudrate = port_data["baudrate"];
    int protocol_version = port_data["protocol_version"];
    auto driver = std::make_shared<DynamixelDriver>();
    if (!driver->init(device_file.c_str(), uint32_t(baudrate))) {
      ROS_ERROR("Error opening serial port %s", device_file.c_str());
      speakError(speak_pub_, "Error opening serial port");
      sleep(1);
      exit(1);
    }
    driver->setPacketHandler(protocol_version);
    std::vector<hardware_interface::RobotHW *> interfaces_on_port;
    // iterate over all devices and ping them to see what is connected to this bus
    std::vector<std::tuple<int, std::string, float, float>> servos_on_port;
    for (std::pair<std::string, int> &device : dxl_devices) {
      std::string name = device.first;
      int id = device.second;
      if (std::find(pinged.begin(), pinged.end(), device.first) != pinged.end()) {
        // we already found this and dont have to search again
      } else {
        ros::NodeHandle dxl_nh(nh, "device_info/" + name);
        int model_number_specified;
        dxl_nh.getParam("model_number", model_number_specified);
        uint16_t model_number_specified_16 = uint16_t(model_number_specified);
        uint16_t *model_number_returned_16 = new uint16_t;
        if (driver->ping(uint8_t(id), model_number_returned_16)) {
          // check if the specified model number matches the actual model number of the device
          if (model_number_specified_16 != *model_number_returned_16) {
            ROS_WARN("Model number of id %d does not match", id);
          }
          // ping was successful, add device correspondingly
          // only add them if the mode is set correspondingly
          if (model_number_specified == 0xAFFE && !only_imu_) {//todo correct number
            // bitfoot
            std::string topic;
            dxl_nh.getParam("topic", topic);
            interfaces_on_port.push_back(new BitFootHardwareInterface(driver, id, topic));
          } else if (model_number_specified == 0xBAFF && !only_pressure_) { //todo correct number
            //IMU
            std::string topic;
            dxl_nh.getParam("topic", topic);
            std::string frame;
            dxl_nh.getParam("frame", frame);
            ImuHardwareInterface *interface = new ImuHardwareInterface(driver, id, topic, frame, name);
            /* Hardware interfaces must be registered at the main RobotHW class.
             * Therefore, a pointer to this class is passed down to the RobotHW classes
             * registering further interfaces */
            interface->setParent(this);
            interfaces_on_port.push_back(interface);
          } else if (model_number_specified == 101 && !only_pressure_) { //todo correct number
            // Buttons
            std::string topic;
            dxl_nh.getParam("topic", topic);
            interfaces_on_port.push_back(new ButtonHardwareInterface(driver, id, topic));
          } else if ((model_number_specified == 311 || model_number_specified == 321 || model_number_specified == 1100)
              && !only_pressure_
              && !only_imu_) {
            // Servos
            float mounting_offset;
            dxl_nh.getParam("mounting_offset", mounting_offset);
            float joint_offset;
            dxl_nh.getParam("joint_offset", joint_offset);
            servos_on_port.push_back(std::make_tuple(id, name, mounting_offset, joint_offset));
          } else if (model_number_specified == 0xABBA) {
            //CORE board
            //TODO
          } else {
            ROS_WARN("Could not identify device for ID %d", id);
          }
          pinged.push_back(name);
        }
      }
    }
    // create a servo bus interface if there were servos found on this bus
    if (servos_on_port.size() > 0) {
      ServoBusInterface *interface = new ServoBusInterface(driver, servos_on_port);
      interfaces_on_port.push_back(interface);
      servo_interface_.addBusInterface(interface);
    }
    // add vector of interfaces on this port to overall collection of interfaces
    interfaces_.push_back(interfaces_on_port);
  }

  if (pinged.size() != dxl_devices.size()) {
    ROS_ERROR("Could not ping all devices!");
    // check which devices were not pinged successful
    for (std::pair<std::string, int> &device : dxl_devices) {
      if (std::find(pinged.begin(), pinged.end(), device.first) != pinged.end()) {
      } else {
        ROS_ERROR("%s with id %d missing", device.first.c_str(), device.second);
      }
    }
  }
}

bool WolfgangHardwareInterface::init(ros::NodeHandle &root_nh) {
  bool success = true;
  // iterate through all ports
  for (std::vector<hardware_interface::RobotHW *> &port_interfaces : interfaces_) {
    // iterate through all interfaces on this port
    for (hardware_interface::RobotHW *interface : port_interfaces) {
      // giving 2 times same node handle to keep interface of base class, dirty
      success &= interface->init(root_nh, root_nh);
    }
  }
  if (success) {
    speakError(speak_pub_, "ros control startup successful");
  } else {
    speakError(speak_pub_, "error starting ros control");
  }
  // init servo interface last after all servo busses are there
  success &= servo_interface_.init(root_nh, root_nh);

  return success;
}

void threaded_read(std::vector<hardware_interface::RobotHW *> &port_interfaces,
                   const ros::Time &t,
                   const ros::Duration &dt) {
  for (hardware_interface::RobotHW *interface : port_interfaces) {
    ROS_WARN("read thread");
    // giving 2 times same node handle to keep interface of base class, dirty
    interface->read(t, dt);
    ROS_WARN("read thread2");
  }
}

void threaded_write(std::vector<hardware_interface::RobotHW *> &port_interfaces,
                    const ros::Time &t,
                    const ros::Duration &dt) {
  for (hardware_interface::RobotHW *interface : port_interfaces) {
    // giving 2 times same node handle to keep interface of base class, dirty
    interface->write(t, dt);
  }
}

void WolfgangHardwareInterface::read(const ros::Time &t, const ros::Duration &dt) {
  for (std::vector<hardware_interface::RobotHW *> &port_interfaces : interfaces_) {
    for (hardware_interface::RobotHW *interface : port_interfaces) {
      interface->read(t, dt);
    }
  }
  servo_interface_.read(t, dt);
  return;
  std::vector<std::thread> threads;
  // start all reads
  for (std::vector<hardware_interface::RobotHW *> port_interfaces : interfaces_) {
    threads.push_back(std::thread(threaded_read, std::ref(port_interfaces), std::ref(t), std::ref(dt)));
  }
  ROS_WARN("Read join");
  // wait for all reads to finish
  for (std::thread &thread : threads) {
    thread.join();
  }
  ROS_WARN("servo read");
  // aggrigate all servo values for controller
  servo_interface_.read(t, dt);
}

void WolfgangHardwareInterface::write(const ros::Time &t, const ros::Duration &dt) {
  return;
  servo_interface_.write(t, dt);
  for (std::vector<hardware_interface::RobotHW *> &port_interfaces : interfaces_) {
    for (hardware_interface::RobotHW *interface : port_interfaces) {
      interface->write(t, dt);
    }
  }
  return;
  // write all controller values to interfaces
  servo_interface_.write(t, dt);

  std::vector<std::thread> threads;
  // start all reads
  for (std::vector<hardware_interface::RobotHW *> port_interfaces : interfaces_) {
    threads.push_back(std::thread(threaded_write, std::ref(port_interfaces), std::ref(t), std::ref(dt)));
  }

  // wait for all reads to finish
  for (std::thread &thread : threads) {
    thread.join();
  }
}
}