#include <rclcpp/rclcpp.hpp>
#include <livelybot_logger/logger_parameters.hpp>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <map>
#include <thread>
#include <chrono>
#include <future>
#include <array>
#include <cmath>
#include <memory>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <livelybot_power/msg/power_switch.hpp>
#include "livelybot_logger/logger_interface.h"
#include "livelybot_logger/msg/logger_operation.hpp"

#define SOCKET_PATH "/tmp/logger_service.sock"
#define MOTOR_COUNT 12

// ---------------------------------------------------------------------------
// Helper: run a shell command with a timeout and return stdout as string.
// ---------------------------------------------------------------------------
static std::string execWithTimeout(const char *cmd, int timeout_ms = 500)
{
    std::string timeout_cmd =
        std::string("timeout ") + std::to_string(timeout_ms / 1000.0) + " " + cmd + " 2>/dev/null";
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(timeout_cmd.c_str(), "r"), pclose);
    if (!pipe) {
        return "";
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------
struct MotorStatus
{
    double position{0};
    double velocity{0};
    double torque{0};
    bool is_valid{false};
};

struct ImuStatus
{
    struct { double x{0}, y{0}, z{0}; } acceleration;
    struct { double x{0}, y{0}, z{0}; } angular_velocity;
    struct { double roll{0}, pitch{0}, yaw{0}; } euler_angles;
    bool is_valid{false};
};

struct PowerStatus
{
    enum class SwitchState { UNKNOWN = -1, OFF = 0, ON = 1 };
    SwitchState system_power{SwitchState::UNKNOWN};
    SwitchState motor_power{SwitchState::UNKNOWN};
    bool is_valid{false};
};

struct BatteryStatus
{
    double voltage{0};
    double current{0};
    double temperature{0};
    double remaining_percentage{0};
    bool is_valid{false};
};

struct NodeStatus
{
    std::string name;
    std::string status;
    std::string cpu;
    std::string memory;

    NodeStatus(std::string n, std::string s, std::string c, std::string m)
        : name(std::move(n)), status(std::move(s)), cpu(std::move(c)), memory(std::move(m)) {}
};

// ---------------------------------------------------------------------------
// LoggerNode class
// ---------------------------------------------------------------------------
class LoggerNode : public rclcpp::Node
{
public:
    LoggerNode()
    : rclcpp::Node("livelybot_logger"),
      system_cpu_usage_(0.0f), system_memory_usage_(0.0f),
      system_disk_usage_(0.0f), cpu_temperature_(0.0f),
      resource_info_valid_(false)
    {
        // Default node-status entries
        for (const auto &n : {"/joy_node", "/joy_teleop", "/livelybot_oled_node", "/power_node",
                               "/livelybot_logger", "/sim2real_master_node", "/yesense_imu_node"}) {
            node_statuses_.emplace_back(n, "unknown", "0%", "0%");
        }

        // Load parameters via generate_parameter_library
        auto param_listener = std::make_shared<livelybot_logger::ParamListener>(
            this->get_node_parameters_interface());
        const auto params = param_listener->get_params();

        const double heartbeat_interval        = params.heartbeat_interval;
        const double power_timeout             = params.power_timeout;
        const double data_acquisition_interval = params.data_acquisition_interval;
        const double status_send_interval      = params.status_send_interval;

        // Unix socket for sending status to external logger service
        initializeSocket();

        // Initialize LoggerInterface with this node
        livelybot_logger::LoggerInterface::init(shared_from_this(), "livelybot_logger");

        sendMessage("OPERATION:livelybot_logger_node_start");

        // Subscriptions
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/error_joint_states", 10,
            std::bind(&LoggerNode::jointStateCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&LoggerNode::imuCallback, this, std::placeholders::_1));

        power_switch_sub_ = create_subscription<livelybot_power::msg::PowerSwitch>(
            "/power_switch_state", 10,
            std::bind(&LoggerNode::powerSwitchCallback, this, std::placeholders::_1));

        bms_error_sub_ = create_subscription<std_msgs::msg::Int8>(
            "/bms_error", 1,
            std::bind(&LoggerNode::bmsErrorCallback, this, std::placeholders::_1));

        battery_volt_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/battery_voltage", 1,
            std::bind(&LoggerNode::batteryVoltCallback, this, std::placeholders::_1));

        battery_curr_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/battery_current", 1,
            std::bind(&LoggerNode::batteryCurrCallback, this, std::placeholders::_1));

        battery_temp_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/battery_temperature", 1,
            std::bind(&LoggerNode::batteryTempCallback, this, std::placeholders::_1));

        operation_sub_ = create_subscription<livelybot_logger::msg::LoggerOperation>(
            "/logger/operation", 10,
            std::bind(&LoggerNode::operationCallback, this, std::placeholders::_1));

        // Timers
        using namespace std::chrono_literals;
        power_timeout_timer_ = create_wall_timer(
            100ms, std::bind(&LoggerNode::powerTimeoutCallback, this));

        data_acquisition_timer_ = create_wall_timer(
            std::chrono::duration<double>(data_acquisition_interval),
            std::bind(&LoggerNode::dataAcquisitionCallback, this));

        status_timer_ = create_wall_timer(
            std::chrono::duration<double>(status_send_interval),
            std::bind(&LoggerNode::statusCallback, this));

        heartbeat_timer_ = create_wall_timer(
            std::chrono::duration<double>(heartbeat_interval),
            std::bind(&LoggerNode::heartbeatCallback, this));

        last_power_update_ = now();

        RCLCPP_INFO(get_logger(),
                    "Logger running — data_acquisition: %.2f s, status_send: %.2f s",
                    data_acquisition_interval, status_send_interval);
    }

    ~LoggerNode() override
    {
        if (socket_fd_ >= 0) {
            close(socket_fd_);
        }
    }

private:
    // -----------------------------------------------------------------------
    // ROS 2 handles
    // -----------------------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            imu_sub_;
    rclcpp::Subscription<livelybot_power::msg::PowerSwitch>::SharedPtr power_switch_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           battery_volt_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           battery_curr_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           battery_temp_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr              bms_error_sub_;
    rclcpp::Subscription<livelybot_logger::msg::LoggerOperation>::SharedPtr operation_sub_;

    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr power_timeout_timer_;
    rclcpp::TimerBase::SharedPtr data_acquisition_timer_;

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------
    MotorStatus motors_[MOTOR_COUNT];
    ImuStatus   imu_;
    PowerStatus power_;
    BatteryStatus battery_;
    rclcpp::Time last_power_update_;

    float system_cpu_usage_;
    float system_memory_usage_;
    float system_disk_usage_;
    float cpu_temperature_;
    bool  resource_info_valid_;

    std::vector<NodeStatus> node_statuses_;

    int  socket_fd_{-1};
    bool socket_initialized_{false};

    // -----------------------------------------------------------------------
    // Unix-socket helpers
    // -----------------------------------------------------------------------
    void initializeSocket()
    {
        socket_fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
        if (socket_fd_ == -1) {
            RCLCPP_ERROR(get_logger(), "Failed to create Unix socket: %s", strerror(errno));
            socket_initialized_ = false;
            return;
        }
        socket_initialized_ = true;
    }

    bool sendMessage(const std::string &message)
    {
        if (!socket_initialized_ || message.empty()) {
            return false;
        }
        if (message.length() > 4000) {
            RCLCPP_WARN(get_logger(), "Message length (%zu) is near socket buffer limit", message.length());
        }

        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

        ssize_t sent = sendto(socket_fd_, message.c_str(), message.length(), 0,
                              reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
        if (sent == -1) {
            RCLCPP_ERROR(get_logger(), "Failed to send socket message: %s", strerror(errno));
            return false;
        }
        if (sent != static_cast<ssize_t>(message.length())) {
            RCLCPP_WARN(get_logger(), "Partial send: %zd of %zu bytes", sent, message.length());
            return false;
        }
        return true;
    }

    // -----------------------------------------------------------------------
    // Subscription callbacks
    // -----------------------------------------------------------------------
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            motors_[i].is_valid = false;
        }
        for (size_t i = 0; i < std::min(msg->position.size(), static_cast<size_t>(MOTOR_COUNT)); i++) {
            motors_[i].position = msg->position[i];
            motors_[i].velocity = msg->velocity[i];
            motors_[i].torque   = msg->effort[i];
            motors_[i].is_valid = true;
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_.acceleration.x = msg->linear_acceleration.x;
        imu_.acceleration.y = msg->linear_acceleration.y;
        imu_.acceleration.z = msg->linear_acceleration.z;

        imu_.angular_velocity.x = msg->angular_velocity.x;
        imu_.angular_velocity.y = msg->angular_velocity.y;
        imu_.angular_velocity.z = msg->angular_velocity.z;

        // Quaternion to RPY via tf2
        tf2::Quaternion q(msg->orientation.x, msg->orientation.y,
                          msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 rot(q);
        rot.getRPY(imu_.euler_angles.roll, imu_.euler_angles.pitch, imu_.euler_angles.yaw);

        imu_.is_valid = true;
    }

    void powerSwitchCallback(const livelybot_power::msg::PowerSwitch::SharedPtr msg)
    {
        last_power_update_ = now();
        power_.system_power = (msg->control_switch == 1) ? PowerStatus::SwitchState::ON
                                                         : PowerStatus::SwitchState::OFF;
        power_.motor_power  = (msg->power_switch == 1)   ? PowerStatus::SwitchState::ON
                                                         : PowerStatus::SwitchState::OFF;
        power_.is_valid = true;
    }

    void batteryVoltCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        battery_.voltage  = msg->data;
        battery_.is_valid = true;
    }

    void batteryCurrCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        battery_.current = msg->data;
    }

    void batteryTempCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        battery_.temperature = msg->data;
    }

    void bmsErrorCallback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        std::ostringstream ss;
        ss << "BMS_ERROR_CODE:" << static_cast<int>(msg->data);
        livelybot_logger::LoggerInterface::logOperation("ERROR", ss.str());
    }

    void operationCallback(const livelybot_logger::msg::LoggerOperation::SharedPtr msg)
    {
        sendMessage("OPERATION:" + msg->operation_type + " | " + msg->operation_data);
    }

    // -----------------------------------------------------------------------
    // Timer callbacks
    // -----------------------------------------------------------------------
    void powerTimeoutCallback()
    {
        const double elapsed = (now() - last_power_update_).seconds();
        if (elapsed > 1.0) {
            power_.is_valid     = false;
            power_.system_power = PowerStatus::SwitchState::UNKNOWN;
            power_.motor_power  = PowerStatus::SwitchState::UNKNOWN;
        }
    }

    void dataAcquisitionCallback()
    {
        updateSystemResourceInfo();
        checkSystemStatus();
    }

    void heartbeatCallback()
    {
        sendMessage("HEARTBEAT");
    }

    void statusCallback()
    {
        const auto start = now();
        try {
            // Update node status with timeout protection
            bool updated = false;
            std::thread t([this, &updated]() {
                try {
                    updateNodeStatusInfo();
                    updated = true;
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(get_logger(), "Node status thread: %s", e.what());
                }
            });

            auto fut = std::async(std::launch::async, [&t]() { t.join(); });
            if (fut.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
                RCLCPP_ERROR(get_logger(), "Node status update timed out");
                t.detach();
            }
            if (!updated) {
                RCLCPP_WARN(get_logger(), "Node status update failed, using previous data");
            }

            battery_.remaining_percentage = 85.0;  // placeholder until BMS reports SoC

            std::ostringstream msg;
            msg << "STATUS:MOTOR;";
            for (int i = 0; i < MOTOR_COUNT; i++) {
                msg << "M" << (i + 1) << ",";
                if (motors_[i].is_valid) {
                    msg << "pos:" << std::fixed << std::setprecision(6) << motors_[i].position << ","
                        << "vel:" << std::fixed << std::setprecision(6) << motors_[i].velocity << ","
                        << "tor:" << std::fixed << std::setprecision(6) << motors_[i].torque;
                } else {
                    msg << "pos:NULL,vel:NULL,tor:NULL";
                }
                if (i < MOTOR_COUNT - 1) { msg << ";"; }
            }

            msg << ";IMU,";
            if (imu_.is_valid) {
                msg << "acc:"  << imu_.acceleration.x  << "/" << imu_.acceleration.y  << "/" << imu_.acceleration.z  << ","
                    << "gyro:" << imu_.angular_velocity.x << "/" << imu_.angular_velocity.y << "/" << imu_.angular_velocity.z << ","
                    << "euler:" << imu_.euler_angles.roll << "/" << imu_.euler_angles.pitch << "/" << imu_.euler_angles.yaw;
            } else {
                msg << "acc:NULL/NULL/NULL,gyro:NULL/NULL/NULL,euler:NULL/NULL/NULL";
            }

            auto switchStr = [](PowerStatus::SwitchState s) -> const char * {
                if (s == PowerStatus::SwitchState::ON)  return "1";
                if (s == PowerStatus::SwitchState::OFF) return "0";
                return "NULL";
            };
            msg << ";POWER,";
            if (power_.is_valid) {
                msg << "sys:"   << switchStr(power_.system_power) << ","
                    << "motor:" << switchStr(power_.motor_power);
            } else {
                msg << "sys:NULL,motor:NULL";
            }

            msg << ";BATTERY,"
                << "volt:"   << (battery_.is_valid ? std::to_string(battery_.voltage)               : "NULL") << ","
                << "curr:"   << (battery_.is_valid ? std::to_string(battery_.current)               : "NULL") << ","
                << "temp:"   << (battery_.is_valid ? std::to_string(battery_.temperature)           : "NULL") << ","
                << "remain:" << (battery_.is_valid ? std::to_string(battery_.remaining_percentage)  : "NULL");

            msg << ";SYSTEM_RESOURCES,";
            if (resource_info_valid_) {
                msg << "cpu:"      << std::fixed << std::setprecision(2) << system_cpu_usage_    << "%,"
                    << "mem:"      << std::fixed << std::setprecision(2) << system_memory_usage_ << "%,"
                    << "disk:"     << std::fixed << std::setprecision(2) << system_disk_usage_   << "%,"
                    << "cpu_temp:" << std::fixed << std::setprecision(2) << cpu_temperature_     << "°C";
            } else {
                msg << "cpu:NULL,mem:NULL,disk:NULL,cpu_temp:NULL";
            }

            for (const auto &ns : node_statuses_) {
                msg << ";ROS_NODE,"
                    << "node:"   << ns.name   << ","
                    << "status:" << ns.status << ","
                    << "cpu:"    << ns.cpu    << ","
                    << "memory:" << ns.memory;
            }

            if (!sendMessage(msg.str())) {
                RCLCPP_ERROR(get_logger(), "Failed to send status message");
            }

            const double ms = (now() - start).seconds() * 1000.0;
            RCLCPP_DEBUG(get_logger(), "statusCallback took %.2f ms", ms);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "statusCallback exception: %s", e.what());
        }
    }

    // -----------------------------------------------------------------------
    // System resource helpers
    // -----------------------------------------------------------------------
    void updateSystemResourceInfo()
    {
        try {
            auto parse_float = [](const std::string &s, float &out) {
                if (!s.empty()) {
                    try { out = std::stof(s); } catch (...) {}
                }
            };

            parse_float(execWithTimeout("top -bn1 | grep 'Cpu(s)' | awk '{print $2 + $4}'", 500),
                        system_cpu_usage_);
            parse_float(execWithTimeout("free | grep Mem | awk '{print $3/$2 * 100.0}'", 500),
                        system_memory_usage_);
            parse_float(execWithTimeout("df -h / | grep / | awk '{print $5}' | sed 's/%//'", 500),
                        system_disk_usage_);

            std::string temp_str = execWithTimeout(
                "cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -nr | head -n1", 500);
            if (!temp_str.empty()) {
                try { cpu_temperature_ = std::stof(temp_str) / 1000.0f; } catch (...) {}
            }

            resource_info_valid_ = true;
        } catch (const std::exception &e) {
            resource_info_valid_ = false;
            RCLCPP_ERROR(get_logger(), "updateSystemResourceInfo: %s", e.what());
        }
    }

    void checkSystemStatus()
    {
        if (battery_.is_valid && battery_.voltage < 10.0) {
            std::ostringstream ss;
            ss << "LOW_BATTERY:" << battery_.voltage << "V";
            livelybot_logger::LoggerInterface::logOperation("ERROR", ss.str());
        }
        if (resource_info_valid_ && cpu_temperature_ > 80.0f) {
            std::ostringstream ss;
            ss << "HIGH_CPU_TEMP:" << cpu_temperature_ << "C";
            livelybot_logger::LoggerInterface::logOperation("ERROR", ss.str());
        }
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (motors_[i].is_valid &&
                std::abs(motors_[i].torque) > 2.0 &&
                std::abs(motors_[i].velocity) < 0.01)
            {
                std::ostringstream ss;
                ss << "MOTOR_STALL:M" << (i + 1)
                   << " torque=" << motors_[i].torque
                   << " velocity=" << motors_[i].velocity;
                livelybot_logger::LoggerInterface::logOperation("ERROR", ss.str());
            }
        }
        if (imu_.is_valid) {
            const double acc_mag = std::sqrt(
                imu_.acceleration.x * imu_.acceleration.x +
                imu_.acceleration.y * imu_.acceleration.y +
                imu_.acceleration.z * imu_.acceleration.z);
            if (acc_mag > 98.0) {
                std::ostringstream ss;
                ss << "HIGH_ACCELERATION:" << acc_mag << "m/s2";
                livelybot_logger::LoggerInterface::logOperation("ERROR", ss.str());
            }
            const double gyro_mag = std::sqrt(
                imu_.angular_velocity.x * imu_.angular_velocity.x +
                imu_.angular_velocity.y * imu_.angular_velocity.y +
                imu_.angular_velocity.z * imu_.angular_velocity.z);
            if (gyro_mag > 8.72) {
                std::ostringstream ss;
                ss << "HIGH_ANGULAR_VELOCITY:" << gyro_mag << "rad/s";
                livelybot_logger::LoggerInterface::logOperation("ERROR", ss.str());
            }
        }
        if (resource_info_valid_) {
            if (system_cpu_usage_ > 90.0f) {
                livelybot_logger::LoggerInterface::logOperation(
                    "ERROR", "HIGH_CPU_USAGE:" + std::to_string(system_cpu_usage_) + "%");
            }
            if (system_memory_usage_ > 90.0f) {
                livelybot_logger::LoggerInterface::logOperation(
                    "ERROR", "HIGH_MEMORY_USAGE:" + std::to_string(system_memory_usage_) + "%");
            }
            if (system_disk_usage_ > 90.0f) {
                livelybot_logger::LoggerInterface::logOperation(
                    "ERROR", "HIGH_DISK_USAGE:" + std::to_string(system_disk_usage_) + "%");
            }
        }
    }

    void updateNodeStatusInfo()
    {
        const std::vector<std::string> target_nodes = {
            "/joy_node", "/joy_teleop", "/livelybot_oled_node", "/power_node",
            "/livelybot_logger", "/sim2real_master_node", "/yesense_imu_node"};

        try {
            const int cpu_cores = std::max(1, static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN)));

            // Use `ros2 node list` instead of `rosnode list`
            std::string node_list = execWithTimeout("ros2 node list", 1000);
            if (node_list.empty()) {
                RCLCPP_WARN(get_logger(), "ros2 node list returned nothing, skipping node status update");
                return;
            }

            node_statuses_.clear();

            std::string all_procs = execWithTimeout("ps aux | grep ros2", 1000);
            std::map<std::string, std::pair<float, float>> process_resources;

            std::istringstream proc_stream(all_procs);
            std::string line;
            while (std::getline(proc_stream, line)) {
                std::istringstream ls(line);
                std::string user, pid, cpu, mem, dummy;
                if (!(ls >> user >> pid >> cpu >> mem)) { continue; }
                for (int i = 0; i < 6; i++) { ls >> dummy; }
                std::string cmd_line;
                std::getline(ls, cmd_line);

                for (const auto &node_name : target_nodes) {
                    std::string basename = (node_name[0] == '/') ? node_name.substr(1) : node_name;
                    if (cmd_line.find(basename) != std::string::npos) {
                        try {
                            float cpu_val = std::stof(cpu) / static_cast<float>(cpu_cores);
                            float mem_val = std::stof(mem);
                            process_resources[node_name] = {cpu_val, mem_val};
                        } catch (...) {}
                        break;
                    }
                }
            }

            for (const auto &node_name : target_nodes) {
                std::string status   = "stopped";
                std::string cpu_str  = "NULL";
                std::string mem_str  = "NULL";

                if (node_list.find(node_name) != std::string::npos) {
                    status = "run";
                    auto it = process_resources.find(node_name);
                    if (it != process_resources.end()) {
                        std::ostringstream c, m;
                        c << std::fixed << std::setprecision(1) << it->second.first  << "%";
                        m << std::fixed << std::setprecision(1) << it->second.second << "%";
                        cpu_str = c.str();
                        mem_str = m.str();
                    }
                }
                node_statuses_.emplace_back(node_name, status, cpu_str, mem_str);
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "updateNodeStatusInfo: %s", e.what());
            node_statuses_.clear();
            for (const auto &n : target_nodes) {
                node_statuses_.emplace_back(n, "unknown", "NULL", "NULL");
            }
        }
    }
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LoggerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
