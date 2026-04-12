#include "robot.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>
#include <unistd.h>

using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;


namespace livelybot_serial
{

bool robot::read_usb_vid_pid(const std::string &device, int &vid, int &pid)
{
    // Extract the device basename (e.g. "ttyACM0" from "/dev/ttyACM0")
    const std::string devname = device.substr(device.rfind('/') + 1);
    const std::string base = "/sys/class/tty/" + devname + "/device/../";

    auto read_hex = [](const std::string &path, int &val) -> bool {
        std::ifstream f(path);
        if (!f.is_open()) { return false; }
        f >> std::hex >> val;
        return !f.fail();
    };

    return read_hex(base + "idVendor", vid) && read_hex(base + "idProduct", pid);
}


robot::robot(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    if (!node_->get_parameter("robot.SDK_version", SDK_version))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get params SDK_version");
        SDK_version = -1;
    }
    if (!node_->get_parameter("robot.serial_baudrate", Seial_baudrate))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get params serial_baudrate");
    }
    if (!node_->get_parameter("robot.robot_name", robot_name))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get params robot_name");
    }
    if (!node_->get_parameter("robot.CANboard_num", CANboard_num))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get params CANboard_num");
    }
    if (!node_->get_parameter("robot.Serial_Type", Serial_Type))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get params Serial_Type");
    }
    if (!node_->get_parameter("robot.control_type", control_type))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get params control_type");
    }

    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32mSDK version: v%s\033[0m", SDK_version_str.c_str());
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32mRobot name: %s\033[0m", robot_name.c_str());
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32mCAN boards: %d\033[0m", CANboard_num);
    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32mSerial type: %s\033[0m", Serial_Type.c_str());

    init_ser();
    error_check_thread_ = std::thread(&robot::check_error, this);

    for (int i = 1; i <= CANboard_num; i++)
    {
        CANboards.push_back(canboard(i, &ser, node_));
    }

    for (canboard &cb : CANboards)
    {
        cb.push_CANport(&CANPorts);
    }
    for (canport *cp : CANPorts)
    {
        cp->puch_motor(&Motors);
    }

    set_port_motor_num();  // configure motor count per port and query firmware version
    if (slave_v >= 4.1f)
    {
        canboard_fdcan_reset();
    }

    if (slave_v < 4.0f)  // detect motor connections (older firmware path)
    {
        fun_v = fun_v1;
        chevk_motor_connection_position();
    }
    else
    {
        fun_v = fun_v2;
        chevk_motor_connection_version();
    }

    if (control_type == 12 && fun_v < fun_v5)
    {
        RCLCPP_ERROR(node_->get_logger(), "Motor firmware is too old for control_type 12.");
    }

    // Diagnostic parameters (overridable via YAML).
    diag_connection_timeout_       = node_->declare_parameter("diagnostics.connection_timeout",       0.5);
    diag_torque_overload_threshold_= node_->declare_parameter("diagnostics.torque_overload_threshold", 10.0);
    diag_torque_overload_duration_ = node_->declare_parameter("diagnostics.torque_overload_duration",  3.0);
    diag_updater_ = std::make_unique<diagnostic_updater::Updater>(node_.get());
    diag_updater_->setHardwareID(robot_name);

    for (motor *m : Motors)
    {
        const std::string name = m->get_motor_name();
        diag_state_.emplace(name, MotorDiagState{});
        // "DS " prefix routes to the Servos group in the diagnostic aggregator.
        diag_updater_->add("DS " + name,
            [this, name, m](diagnostic_updater::DiagnosticStatusWrapper &stat) {
                motorDiagnostic(name, m, stat);
            });
    }

    publish_joint_state = true;
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    pub_thread_ = std::thread(&robot::publishJointStates, this);

    joint_cmd_sub_ = node_->create_subscription<bitbots_msgs::msg::JointCommand>(
        "joint_command", 1,
        std::bind(&robot::jointCommandCallback, this, std::placeholders::_1));

    torque_sub_ = node_->create_subscription<bitbots_msgs::msg::JointTorque>(
        "set_torque_individual", 10,
        std::bind(&robot::torqueCallback, this, std::placeholders::_1));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    RCLCPP_INFO(node_->get_logger(),
                "\033[1;32mRobot has %ld motors\033[0m", Motors.size());
    RCLCPP_INFO(node_->get_logger(), "Robot init complete.");
}


robot::~robot()
{
    publish_joint_state = false;
    set_stop();
    motor_send_2();
    motor_send_2();
    for (auto &thread : ser_recv_threads)
    {
        if (thread.joinable())
            thread.join();
    }

    if (pub_thread_.joinable())
    {
        pub_thread_.join();
    }

    if (error_check_thread_.joinable())
    {
        error_check_thread_.join();
    }
}


void robot::publishJointStates()
{
    rclcpp::Rate rate(100);
    while (publish_joint_state && rclcpp::ok())
    {
        sensor_msgs::msg::JointState js;
        js.header.stamp = node_->now();

        const double now_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        for (motor *m : Motors)
        {
            motor_back_t *data_ptr = m->get_current_motor_state();

            // Drop motors with no recent data entirely — consumers must not
            // assume every motor is always present in the message.
            //if (data_ptr->time == 0.0 || now_sec - data_ptr->time > 0.1)
            //    continue;

            js.name.push_back(m->get_motor_name());
            js.position.push_back(data_ptr->position);
            js.velocity.push_back(data_ptr->velocity);
            js.effort.push_back(data_ptr->torque);
        }

        joint_state_pub_->publish(js);
        rate.sleep();
    }
}


void robot::jointCommandCallback(bitbots_msgs::msg::JointCommand::ConstSharedPtr msg)
{
    if (msg->joint_names.empty())
        return;

    for (size_t i = 0; i < msg->joint_names.size(); ++i)
    {
        const std::string &name = msg->joint_names[i];

        // Skip motors that are currently torque-off (puppeteering / teach mode).
        {
            std::lock_guard<std::mutex> lock(torque_off_mutex_);
            if (torque_off_motors_.count(name))
                continue;
        }

        // Find the motor whose name matches this joint.
        motor *m = nullptr;
        for (motor *candidate : Motors)
        {
            if (candidate->get_motor_name() == name)
            {
                m = candidate;
                break;
            }
        }

        if (!m)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                  "JointCommand: joint '%s' not found — ignoring", name.c_str());
            continue;
        }

        const float pos = static_cast<float>(msg->positions[i]);
        const float vel = (i < msg->velocities.size() && msg->velocities[i] > 0.0)
                          ? static_cast<float>(msg->velocities[i]) : 0.0f;
        const float max_tqe = (i < msg->max_torques.size() && msg->max_torques[i] > 0.0)
                               ? static_cast<float>(msg->max_torques[i]) : -1.0f;

        if (max_tqe > 0.0f)
            m->pos_vel_MAXtqe(pos, vel, max_tqe);
        else
            m->position(pos);
    }

    motor_send_2();
}


void robot::torqueCallback(bitbots_msgs::msg::JointTorque::ConstSharedPtr msg)
{
    if (msg->joint_names.size() != msg->on.size())
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "JointTorque: joint_names length (%zu) != on length (%zu) — ignoring",
                     msg->joint_names.size(), msg->on.size());
        return;
    }

    bool need_send = false;

    for (size_t i = 0; i < msg->joint_names.size(); ++i)
    {
        const std::string &name = msg->joint_names[i];

        // Find matching motor.
        motor *m = nullptr;
        for (motor *candidate : Motors)
        {
            if (candidate->get_motor_name() == name)
            {
                m = candidate;
                break;
            }
        }
        if (!m)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                  "JointTorque: joint '%s' not found — ignoring", name.c_str());
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(torque_off_mutex_);
            if (msg->on[i])
            {
                torque_off_motors_.erase(name);
            }
            else
            {
                torque_off_motors_.insert(name);
                // Send zero-torque so the motor is immediately back-driveable.
                m->torque(0.0f);
                need_send = true;
            }
        }
    }

    if (need_send)
        motor_send_2();
}


void robot::motorDiagnostic(const std::string &name, motor *m,
                             diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stat.hardware_id = name;
    motor_back_t *d = m->get_current_motor_state();

    const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    // --- connection check ---
    const double age = now - d->time;
    if (d->time == 0.0 || age > diag_connection_timeout_)
    {
        stat.summary(DiagStatus::STALE, "No CAN data");
        stat.add("last_update_age_s", age);
        return;
    }

    stat.summary(DiagStatus::OK, "OK");

    // --- fault code ---
    if (d->fault != 0)
    {
        std::ostringstream ss;
        ss << "Fault 0x" << std::hex << std::setw(2) << std::setfill('0')
           << static_cast<int>(d->fault);
        stat.mergeSummary(DiagStatus::ERROR, ss.str());
    }

    // --- torque overload (sustained) ---
    auto &ds = diag_state_[name];
    if (std::fabs(d->torque) > diag_torque_overload_threshold_)
    {
        if (ds.torque_high_since == 0.0)
            ds.torque_high_since = now;
        else if (now - ds.torque_high_since >= diag_torque_overload_duration_)
            stat.mergeSummary(DiagStatus::WARN,
                              "Torque overload (" +
                              std::to_string(static_cast<int>(d->torque)) + " N·m)");
    }
    else
    {
        ds.torque_high_since = 0.0;
    }

    // --- values ---
    stat.add("torque_nm",      d->torque);
    stat.add("fault_code",     static_cast<int>(d->fault));
    stat.add("mode",           static_cast<int>(d->mode));
}

void robot::detect_motor_limit()
{
    if (!motor_position_limit_flag && !motor_torque_limit_flag)
    {
        for (motor *m : Motors)
        {
            if (m->pos_limit_flag)
            {
                RCLCPP_ERROR(node_->get_logger(), "Robot position limit reached — stopping motors.");
                set_stop();
                motor_position_limit_flag = m->pos_limit_flag;
                break;
            }

            if (m->tor_limit_flag)
            {
                RCLCPP_ERROR(node_->get_logger(), "Robot torque limit reached — stopping motors.");
                set_stop();
                motor_torque_limit_flag = m->tor_limit_flag;
                break;
            }
        }
    }
}


void robot::motor_send_2()
{
    for (motor *m : Motors)
    {
        m->pos_vel_tqe_kp_kd(m->get_current_motor_state()->position, 0, 0, 10, 1);
    }

    if (!motor_position_limit_flag && !motor_torque_limit_flag)
    {
        for (canboard &cb : CANboards)
        {
            cb.motor_send_2();
        }
    }
}


int robot::serial_pid_vid(const char *name, int *pid, int *vid)
{
    if (!read_usb_vid_pid(std::string(name), *vid, *pid))
    {
        return 1;
    }
    std::cout << "Port: " << name
              << ", PID: 0x" << std::hex << *pid
              << ", VID: 0x" << *vid << std::dec << std::endl;
    return 0;
}


int robot::serial_pid_vid(const char *name)
{
    int vid = 0, pid = 0;
    if (!read_usb_vid_pid(std::string(name), vid, pid))
    {
        return -1;
    }

    if (pid == 0xFFFF)
    {
        switch (vid)
        {
        case 0xCAF1:
            return 4;
        case 0xCAE1:
            return 7;
        default:
            return -3;
        }
    }

    return -1;
}


std::vector<std::string> robot::list_serial_ports(const std::string &full_prefix)
{
    const std::string base_path = full_prefix.substr(0, full_prefix.rfind('/') + 1);
    const std::string prefix = full_prefix.substr(full_prefix.rfind('/') + 1);
    std::vector<std::string> serial_ports;

    DIR *directory = opendir(base_path.c_str());
    if (!directory)
    {
        std::cerr << "Could not open directory " << base_path << std::endl;
        return serial_ports;
    }

    struct dirent *entry;
    while ((entry = readdir(directory)) != nullptr)
    {
        const std::string entry_name = entry->d_name;
        if (entry_name.find(prefix) == 0)
        {
            serial_ports.push_back(base_path + entry_name);
        }
    }

    closedir(directory);
    std::sort(serial_ports.begin(), serial_ports.end());
    return serial_ports;
}


void robot::init_ser()
{
    ser.clear();
    ser_recv_threads.clear();
    str.clear();

    std::vector<std::string> ports = list_serial_ports(Serial_Type);
    std::cout << "Serial port list:" << std::endl;
    int8_t board_port_num = 99;
    for (const std::string &port : ports)
    {
        const int8_t r = serial_pid_vid(port.c_str());
        if (r > 0)
        {
            RCLCPP_INFO(node_->get_logger(), "Serial port %ld = %s", str.size(), port.c_str());
            str.push_back(port);
            board_port_num = r > board_port_num ? board_port_num : r;
        }
    }

    if (board_port_num == 0x63)  // 99 — no valid port found
    {
        RCLCPP_ERROR(node_->get_logger(), "No communication board detected!");
        exit(-1);
    }

    const uint8_t port_max_num = board_port_num * CANboard_num;
    if (str.size() < port_max_num)
    {
        RCLCPP_INFO(node_->get_logger(), "Expected port count: %d", port_max_num);
        RCLCPP_ERROR(node_->get_logger(),
                     "Fewer communication board serial ports detected than expected.");
        exit(-1);
    }

    for (int cb_id = 1; cb_id <= CANboard_num; cb_id++)
    {
        int cp_num = 0;
        const std::string board_key = "robot.CANboard.No_" + std::to_string(cb_id) + "_CANboard.CANport_num";
        if (node_->get_parameter(board_key, cp_num))
        {
            RCLCPP_INFO(node_->get_logger(), "Board %d has %d port(s)", cb_id, cp_num);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get CANport_num for board %d", cb_id);
            exit(-1);
        }

        std::vector<int> serial_id_used;
        for (int cp_id = 1; cp_id <= cp_num; cp_id++)
        {
            int serial_id = 0;
            const std::string ser_key = "robot.CANboard.No_" + std::to_string(cb_id) +
                                        "_CANboard.CANport.CANport_" + std::to_string(cp_id) +
                                        ".serial_id";
            if (node_->get_parameter(ser_key, serial_id))
            {
                if (serial_id > static_cast<int>(str.size()) || serial_id < 1)
                {
                    RCLCPP_ERROR(node_->get_logger(), "serial_id %d is out of range!", serial_id);
                    exit(-1);
                }
                if (std::find(serial_id_used.begin(), serial_id_used.end(), serial_id)
                    != serial_id_used.end())
                {
                    RCLCPP_ERROR(node_->get_logger(), "Duplicate serial_id %d!", serial_id);
                    exit(-1);
                }
                serial_id_used.push_back(serial_id);

                lively_serial *s = new lively_serial(&str[serial_id - 1], Seial_baudrate);
                ser.push_back(s);
                ser_recv_threads.push_back(std::thread(&lively_serial::recv_1for6_42, s));
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Failed to get serial_id for board %d port %d", cb_id, cp_id);
            }
        }
    }
}


typedef enum
{
    error_check = 0,    // normal operation
    error_clear,        // error detected, cleaning up
    error_wait_dev,     // waiting for device to reappear
    error_reconnect,    // reconnecting
} error_run_state_e;


void robot::check_error()
{
    while (true)
    {
        static error_run_state_e last_state = error_reconnect;
        static error_run_state_e state = error_check;

        switch (state)
        {
        case error_check:
        {
            bool serial_error = false;
            for (lively_serial *s : ser)
            {
                if (s->is_serial_error())
                {
                    serial_error = true;
                    break;
                }
            }
            if (serial_error)
            {
                state = error_clear;
                std::cerr << "Serial error detected." << std::endl;
            }
        }
        break;

        case error_clear:
        {
            for (lively_serial *s : ser)
            {
                s->set_run_flag(false);
            }
            for (auto &_thread : ser_recv_threads)
            {
                if (_thread.joinable())
                {
                    _thread.join();
                }
            }

            CANboards.clear();
            CANPorts.clear();
            Motors.clear();

            for (lively_serial *s : ser)
            {
                delete s;
            }

            ser.clear();
            state = error_wait_dev;
            std::cerr << "Objects and threads cleared." << std::endl;
        }
        break;

        case error_wait_dev:
        {
            const int exist_num = check_serial_dev_exist(8);
            std::cerr << "Found " << exist_num << " device(s)." << std::endl;
            if (exist_num < 4)
            {
                std::cerr << "Cannot find 4 motor serial ports — check USB connections." << std::endl;
            }
            else
            {
                std::cout << "All devices found." << std::endl;
                state = error_reconnect;
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            }
        }
        break;

        case error_reconnect:
        {
            std::cerr << "Reconnect start." << std::endl;
            init_ser();
            for (int i = 1; i <= CANboard_num; i++)
            {
                CANboards.push_back(canboard(i, &ser, node_));
            }
            for (canboard &cb : CANboards)
            {
                cb.push_CANport(&CANPorts);
            }
            for (canport *cp : CANPorts)
            {
                cp->puch_motor(&Motors);
            }
            set_port_motor_num();
            chevk_motor_connection_version();
            state = error_check;
            std::cerr << "Reconnect complete." << std::endl;
        }
        break;

        default:
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if (state != last_state)
        {
            last_state = state;
            std::cout << "error_run_state = " << state << std::endl;
        }
    }
}


int robot::check_serial_dev_exist(int file_num)
{
    int exist_num = 0;
    std::cout << "Checking serial device existence..." << std::endl;

    for (int i = 0; i < file_num; i++)
    {
        const std::string dev = "/dev/ttyACM" + std::to_string(i);
        if (access(dev.c_str(), F_OK) == 0)
        {
            exist_num++;
            std::cout << "Exists: " << dev << std::endl;
        }
    }

    std::cout << "Devices found: " << exist_num << std::endl;
    return exist_num;
}


void robot::set_port_motor_num()
{
    for (canboard &cb : CANboards)
    {
        slave_v = cb.set_port_motor_num();
    }
}


void robot::send_get_motor_state_cmd()
{
    if (fun_v >= fun_v4)
    {
        for (canboard &cb : CANboards)
        {
            cb.send_get_motor_state_cmd2();
        }
    }
    else if (fun_v >= fun_v2 || control_type == 0)
    {
        for (canboard &cb : CANboards)
        {
            cb.send_get_motor_state_cmd();
        }
    }
    else
    {
        for (motor *m : Motors)
        {
            m->velocity(0.0f);
        }
        motor_send_2();
    }
}


void robot::send_get_motor_version_cmd()
{
    if (slave_v < 4.0f)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "The current communication board does not support this function!");
        exit(-1);
    }

    for (canboard &cb : CANboards)
    {
        cb.send_get_motor_version_cmd();
    }
}


void robot::motor_version_detection()
{
    uint16_t v_old = 0xFFFF;
    uint16_t i = 0;

    RCLCPP_INFO(node_->get_logger(), "--------------- Motor versions ---------------");
    for (motor *m : Motors)
    {
        const auto v = m->get_version();
        RCLCPP_INFO(node_->get_logger(), "motors[%02d]: id:%02d v%d.%d.%d",
                    i++, v.id, v.major, v.minor, v.patch);
        const uint16_t v_new = (v.major << 12) | (v.minor << 4) | v.patch;
        if (v_old > v_new && v_new != 0)
        {
            v_old = v_new;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "----------------------------------------------");

    if (v_old >= COMBINE_VERSION(4, 4, 6))
    {
        fun_v = fun_v5;
    }
    else if (v_old >= COMBINE_VERSION(4, 2, 3))
    {
        fun_v = fun_v4;
    }
    else if (v_old >= COMBINE_VERSION(4, 2, 2))
    {
        fun_v = fun_v3;
    }
    else if (v_old >= COMBINE_VERSION(4, 2, 0))
    {
        fun_v = fun_v2;
    }
    else
    {
        fun_v = fun_v1;
    }

    for (canboard &cb : CANboards)
    {
        cb.set_fun_v(fun_v);
    }

    RCLCPP_INFO(node_->get_logger(), "fun_v = %d", fun_v);

    uint8_t v = 0;
    uint8_t v2 = 0;
    for (motor *m : Motors)
    {
        v = m->get_version().major;

        if (v == 5 && v2 != 0 && v != v2)
        {
            RCLCPP_ERROR(node_->get_logger(), "Inconsistent motor firmware versions!");
            exit(-1);
        }

        v2 = v;

        if (v == 5)
        {
            m->set_type(mGeneral);
        }
    }
}


void robot::set_data_reset()
{
    if (fun_v < fun_v3)
    {
        RCLCPP_ERROR(node_->get_logger(), "Current feature version does not support set_data_reset!");
        exit(-3);
    }

    for (canboard &cb : CANboards)
    {
        cb.set_data_reset();
    }
}


void robot::chevk_motor_connection_version()
{
    int t = 0;
    int num = 0;
    std::vector<int> board;
    std::vector<int> port;
    std::vector<int> id;

    RCLCPP_INFO(node_->get_logger(), "Detecting motor connections...");
    while (t++ < 20)
    {
        send_get_motor_version_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        num = 0;
        board.clear();
        port.clear();
        id.clear();
        for (motor *m : Motors)
        {
            cdc_rx_motor_version_s &v = m->get_version();
            if (v.major != 0)
            {
                ++num;
            }
            else
            {
                board.push_back(m->get_motor_belong_canboard());
                port.push_back(m->get_motor_belong_canport());
                id.push_back(m->get_motor_id());
            }
        }

        if (num == static_cast<int>(Motors.size()))
        {
            break;
        }

        if (t % 10 == 0)
        {
            RCLCPP_INFO(node_->get_logger(), "Waiting for motor connection...");
        }
    }

    if (num == static_cast<int>(Motors.size()))
    {
        RCLCPP_INFO(node_->get_logger(), "\033[1;32mAll motor connections are normal.\033[0m");
    }
    else
    {
        for (int i = 0; i < static_cast<int>(Motors.size()) - num; i++)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "CANboard(%d) CANport(%d) id(%d) — motor disconnected!",
                         board[i], port[i], id[i]);
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    motor_version_detection();
}


void robot::chevk_motor_connection_position()
{
    int t = 0;
    int num = 0;
    std::vector<int> board;
    std::vector<int> port;
    std::vector<int> id;

    const int max_delay_ms = 2000;

    RCLCPP_INFO(node_->get_logger(), "Detecting motor connections...");
    while (t++ < max_delay_ms)
    {
        send_get_motor_state_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        num = 0;
        board.clear();
        port.clear();
        id.clear();
        for (motor *m : Motors)
        {
            if (m->get_current_motor_state()->position != 999.0f)
            {
                ++num;
            }
            else
            {
                board.push_back(m->get_motor_belong_canboard());
                port.push_back(m->get_motor_belong_canport());
                id.push_back(m->get_motor_id());
            }
        }

        if (num == static_cast<int>(Motors.size()))
        {
            break;
        }

        if (t % 1000 == 0)
        {
            RCLCPP_INFO(node_->get_logger(), "Waiting for motor state...");
        }
    }

    if (num == static_cast<int>(Motors.size()))
    {
        RCLCPP_INFO(node_->get_logger(), "\033[1;32mAll motor connections are normal.\033[0m");
    }
    else
    {
        for (int i = 0; i < static_cast<int>(Motors.size()) - num; i++)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "CANboard(%d) CANport(%d) id(%d) — motor disconnected!",
                         board[i], port[i], id[i]);
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
}


void robot::set_stop()
{
    for (canboard &cb : CANboards)
    {
        cb.set_stop();
    }
    motor_send_2();
}


void robot::set_reset()
{
    for (canboard &cb : CANboards)
    {
        cb.set_reset();
    }
}


void robot::set_reset_zero()
{
    for (canboard &cb : CANboards)
    {
        cb.set_reset_zero();
    }
}


void robot::set_reset_zero(std::initializer_list<int> motors)
{
    for (auto const &motor_idx : motors)
    {
        const int board_id = Motors[motor_idx]->get_motor_belong_canboard() - 1;
        const int port_id  = Motors[motor_idx]->get_motor_belong_canport() - 1;
        const int motor_id = Motors[motor_idx]->get_motor_id();
        RCLCPP_INFO(node_->get_logger(), "Motor index=%d board=%d port=%d id=%d",
                    motor_idx, board_id, port_id, motor_id);

        if (CANPorts[port_id]->set_conf_load(motor_id) != 0)
        {
            RCLCPP_ERROR(node_->get_logger(), "Motor %d settings restoration failed.", motor_idx);
            return;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "Motor %d settings restored. Initiating zero position reset.", motor_idx);
        if (CANPorts[port_id]->set_reset_zero(motor_id) == 0)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Motor %d zero position reset successfully.", motor_idx);
            if (CANPorts[port_id]->set_conf_write(motor_id) == 0)
            {
                RCLCPP_INFO(node_->get_logger(), "Motor %d settings saved.", motor_idx);
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Motor %d settings save failed.", motor_idx);
            }
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Motor %d zero position reset failed.", motor_idx);
        }
    }
}


void robot::set_motor_runzero()
{
    for (int i = 0; i < 5; i++)
    {
        for (canboard &cb : CANboards)
        {
            cb.set_motor_runzero();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::seconds(4));
}


void robot::set_timeout(int16_t t_ms)
{
    for (int i = 0; i < 5; i++)
    {
        for (canboard &cb : CANboards)
        {
            cb.set_time_out(t_ms);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


void robot::set_timeout(uint8_t portx, int16_t t_ms)
{
    for (int i = 0; i < 5; i++)
    {
        CANboards[0].set_time_out(portx, t_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


void robot::canboard_bootloader()
{
    for (canboard &cb : CANboards)
    {
        cb.canboard_bootloader();
    }
}


void robot::canboard_fdcan_reset()
{
    RCLCPP_INFO(node_->get_logger(), "CAN board FDCAN reset.");
    for (canboard &cb : CANboards)
    {
        cb.canboard_fdcan_reset();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

}  // namespace livelybot_serial
