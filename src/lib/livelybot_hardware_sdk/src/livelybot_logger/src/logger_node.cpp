#include <ros/ros.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <livelybot_logger/LoggerStatus.h>
#include <livelybot_logger/LoggerOperation.h>
#include "livelybot_logger/logger_interface.h"
#include <iomanip>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <livelybot_power/Power_switch.h>
#include <std_msgs/Float32.h>
#include <cstdio>          // for popen
#include <memory>          // for std::unique_ptr
#include <stdexcept>       // for std::runtime_error
#include <array>           // for std::array
#include <algorithm>       // for std::remove
#include <vector>          // for std::vector
#include <map>             // for std::map
#include <regex>           // for std::regex
#include <thread>          // for std::thread
#include <chrono>          // for std::chrono
#include <future>          // for std::async
#include <ctime>           // for time
#include <std_msgs/Int8.h> // 添加 std_msgs/Int8 头文件

using namespace livelybot_logger; // 添加命名空间

#define SOCKET_PATH "/tmp/logger_service.sock"
#define MOTOR_COUNT 12
// 执行系统命令并返回结果的辅助函数
std::string execSystemCommand(const char *cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
    {
        return "";
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}

// 带超时的系统命令执行函数
std::string execSystemCommandWithTimeout(const char *cmd, int timeout_ms = 500)
{
    // 添加超时参数到命令
    std::string timeout_cmd = std::string("timeout ") + std::to_string(timeout_ms / 1000.0) + " " + cmd + " 2>/dev/null";

    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(timeout_cmd.c_str(), "r"), pclose);
    if (!pipe)
    {
        ROS_WARN("Command execution failed: %s", cmd);
        return "";
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}

struct MotorStatus
{
    double position;
    double velocity;
    double torque;
    bool is_valid; // 添加数据有效性标志

    MotorStatus() : position(0), velocity(0), torque(0), is_valid(false) {} // 构造函数初始化
};

struct ImuStatus
{
    struct
    {
        double x, y, z;
    } acceleration;

    struct
    {
        double x, y, z;
    } angular_velocity;

    struct
    {
        double roll, pitch, yaw;
    } euler_angles;

    bool is_valid; // 添加数据有效性标志

    ImuStatus() : is_valid(false) {} // 构造函数初始化
};

struct PowerStatus
{
    enum class SwitchState
    {
        UNKNOWN = -1,
        OFF = 0,
        ON = 1
    };

    SwitchState system_power;
    SwitchState motor_power;
    bool is_valid;

    PowerStatus() : system_power(SwitchState::UNKNOWN),
                    motor_power(SwitchState::UNKNOWN),
                    is_valid(false) {}
};

struct BatteryStatus
{
    double voltage;
    double current;
    double temperature;
    double remaining_percentage;
    bool is_valid; // 添加数据有效性标志

    BatteryStatus() : voltage(0), current(0), temperature(0), remaining_percentage(0), is_valid(false) {} // 添加构造函数
};

// 添加ROS节点资源数据结构
struct NodeStatus
{
    std::string name;
    std::string status;
    std::string cpu;
    std::string memory;

    NodeStatus(const std::string &n, const std::string &s, const std::string &c, const std::string &m)
        : name(n), status(s), cpu(c), memory(m) {}
};

class LoggerNode
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber imu_sub_;
    ros::Timer status_timer_;
    ros::Timer heartbeat_timer_;
    ros::Timer power_timeout_timer_;
    ros::Timer data_acquisition_timer_; // 新增：快速数据获取定时器
    ros::Subscriber operation_sub_;
    ros::Subscriber power_switch_sub_; // 添加功率板开关状态订阅者
    ros::Subscriber battery_volt_sub_; // 添加电池电压订阅器
    ros::Subscriber battery_curr_sub_; // 添加电池电流订阅器
    ros::Subscriber battery_temp_sub_; // 添加电池温度订阅器
    ros::Subscriber bms_error_sub_;    // 新增：bms_error 话题订阅者
    int socket_fd_;
    bool socket_initialized_; // 添加socket初始化状态标志

    // 状态数据
    MotorStatus motors_[MOTOR_COUNT];
    ImuStatus imu_;
    PowerStatus power_;
    BatteryStatus battery_;
    ros::Time last_power_update_; // 添加最后更新时间

    // 系统资源使用数据
    float system_cpu_usage_;
    float system_memory_usage_;
    float system_disk_usage_;
    float cpu_temperature_; // 添加CPU温度变量
    bool resource_info_valid_;

    // ROS节点状态数据
    std::vector<NodeStatus> node_statuses_;

    // 参数
    double heartbeat_interval_;
    double power_timeout_;             // 添加电源状态超时时间
    double data_acquisition_interval_; // 新增：数据获取间隔
    double status_send_interval_;      // 新增：状态发送间隔

    void initializeSocket()
    {
        socket_fd_ = socket(AF_UNIX, SOCK_DGRAM, 0);
        if (socket_fd_ == -1)
        {
            ROS_ERROR("Failed to create socket");
            socket_initialized_ = false;
            return;
        }
        socket_initialized_ = true;
    }

    bool sendMessage(const std::string &message)
    {
        if (!socket_initialized_ || message.empty())
        {
            ROS_ERROR("Cannot send message: socket not initialized or empty message");
            return false;
        }

        // 添加消息大小检查
        if (message.length() > 4000)
        {
            ROS_WARN("Message length (%zu bytes) is close to BUFFER_SIZE limit!", message.length());
        }

        struct sockaddr_un addr;
        memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

        ssize_t sent = sendto(socket_fd_, message.c_str(), message.length(), 0,
                              (struct sockaddr *)&addr, sizeof(addr));

        if (sent == -1)
        {
            ROS_ERROR("Failed to send message: %s, error: %s",
                      message.c_str(), strerror(errno));
            return false;
        }
        else if (sent != static_cast<ssize_t>(message.length()))
        {
            ROS_WARN("Partial send: %zd of %zu bytes", sent, message.length());
            return false;
        }

        return true;
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        // 先将所有电机数据标记为无效
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            motors_[i].is_valid = false;
        }

        // 更新收到的数据,并标记为有效
        for (size_t i = 0; i < std::min(msg->position.size(), (size_t)MOTOR_COUNT); i++)
        {
            motors_[i].position = msg->position[i];
            motors_[i].velocity = msg->velocity[i];
            motors_[i].torque = msg->effort[i];
            motors_[i].is_valid = true;
        }
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        // 更新加速度数据
        imu_.acceleration.x = msg->linear_acceleration.x;
        imu_.acceleration.y = msg->linear_acceleration.y;
        imu_.acceleration.z = msg->linear_acceleration.z;

        // 更新角速度数据
        imu_.angular_velocity.x = msg->angular_velocity.x;
        imu_.angular_velocity.y = msg->angular_velocity.y;
        imu_.angular_velocity.z = msg->angular_velocity.z;

        // 从四元数转换为欧拉角
        tf::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf::Matrix3x3 rot_mat(q);
        rot_mat.getRPY(imu_.euler_angles.roll,
                       imu_.euler_angles.pitch,
                       imu_.euler_angles.yaw);

        // 标记数据为有效
        imu_.is_valid = true;
    }

    void powerTimeoutCallback(const ros::TimerEvent &)
    {
        ros::Time now = ros::Time::now();
        if ((now - last_power_update_).toSec() > power_timeout_)
        {
            power_.is_valid = false;
            power_.system_power = PowerStatus::SwitchState::UNKNOWN;
            power_.motor_power = PowerStatus::SwitchState::UNKNOWN;
            ROS_DEBUG("Power status timeout - marking as invalid");
        }
    }

    void powerSwitchCallback(const livelybot_power::Power_switch::ConstPtr &msg)
    {
        // 更新最后接收时间
        last_power_update_ = ros::Time::now();

        // 更新电源状态
        power_.system_power = (msg->control_switch == 1) ? PowerStatus::SwitchState::ON : PowerStatus::SwitchState::OFF;
        power_.motor_power = (msg->power_switch == 1) ? PowerStatus::SwitchState::ON : PowerStatus::SwitchState::OFF;
        power_.is_valid = true;

        // 注释掉调试输出
        // ROS_INFO("Logger接收到功率板开关状态 - 控制开关: %d (转换为: %s), 电源开关: %d (转换为: %s)",
        //        msg->control_switch,
        //        (power_.system_power == PowerStatus::SwitchState::ON) ? "ON" : "OFF",
        //        msg->power_switch,
        //        (power_.motor_power == PowerStatus::SwitchState::ON) ? "ON" : "OFF");
    }

    void batteryVoltCallback(const std_msgs::Float32::ConstPtr &msg)
    {
        battery_.voltage = msg->data;
        battery_.is_valid = true;
    }

    void batteryCurrCallback(const std_msgs::Float32::ConstPtr &msg)
    {
        battery_.current = msg->data;
        // battery_.is_valid标志已经在电压回调中设置
    }

    void batteryTempCallback(const std_msgs::Float32::ConstPtr &msg)
    {
        battery_.temperature = msg->data;
        // battery_.is_valid标志已经在电压回调中设置
    }

    // 实现 bms_error 回调函数
    void bmsErrorCallback(const std_msgs::Int8::ConstPtr &msg)
    {
        std::ostringstream data_stream;
        data_stream << "BMS_ERROR_CODE: " << static_cast<int>(msg->data);
        livelybot_logger::LoggerInterface::logOperation("ERROR", data_stream.str());
    }

    // 新增：数据获取回调函数，用于高频率获取和检查状态数据
    void dataAcquisitionCallback(const ros::TimerEvent &)
    {
        // 确保处理所有待处理的回调
        ros::spinOnce();

        // 更新系统资源信息（但不一定每次都发送）
        updateSystemResourceInfo();

        // 检查系统状态并在需要时发送警报
        checkSystemStatus();
    }

    // 实现系统状态检查函数
    void checkSystemStatus()
    {
        // 检查电池电压
        if (battery_.is_valid && battery_.voltage < 10.0)
        { // 假设10.0V是低电压阈值
            // 使用LoggerInterface发送低电压警报
            std::ostringstream data_stream;
            data_stream << "LOW_BATTERY:" << battery_.voltage << "V";
            LoggerInterface::logOperation("ERROR", data_stream.str());
        }

        // 检查CPU温度
        if (resource_info_valid_ && cpu_temperature_ > 80.0)
        { // 假设80°C是高温阈值
            // 使用LoggerInterface发送CPU高温警报
            std::ostringstream data_stream;
            data_stream << "HIGH_CPU_TEMP:" << cpu_temperature_ << "°C";
            LoggerInterface::logOperation("ERROR", data_stream.str());
        }

        // 检查电机状态（示例：检查电机是否卡死）
        for (int i = 0; i < MOTOR_COUNT; i++)
        {
            if (motors_[i].is_valid && std::abs(motors_[i].torque) > 2.0 && std::abs(motors_[i].velocity) < 0.01)
            {
                // 使用LoggerInterface发送电机可能卡死的警报
                std::ostringstream data_stream;
                data_stream << "MOTOR_STALL:M" << (i + 1) << " torque=" << motors_[i].torque
                            << " velocity=" << motors_[i].velocity;
                LoggerInterface::logOperation("ERROR", data_stream.str());
            }
        }

        // 检查IMU数据异常
        if (imu_.is_valid)
        {
            // 检查加速度异常 - 例如超过10g可能表示碰撞或坠落
            double acc_magnitude = sqrt(pow(imu_.acceleration.x, 2) +
                                        pow(imu_.acceleration.y, 2) +
                                        pow(imu_.acceleration.z, 2));
            if (acc_magnitude > 98.0)
            { // 10g ≈ 98 m/s²
                std::ostringstream data_stream;
                data_stream << "HIGH_ACCELERATION:" << acc_magnitude << "m/s²";
                LoggerInterface::logOperation("ERROR", data_stream.str());
            }

            // 检查角速度异常 - 例如超过500°/s可能表示不受控制的旋转
            double gyro_magnitude = sqrt(pow(imu_.angular_velocity.x, 2) +
                                         pow(imu_.angular_velocity.y, 2) +
                                         pow(imu_.angular_velocity.z, 2));
            if (gyro_magnitude > 8.72)
            { // 500°/s ≈ 8.72 rad/s
                std::ostringstream data_stream;
                data_stream << "HIGH_ANGULAR_VELOCITY:" << gyro_magnitude << "rad/s";
                LoggerInterface::logOperation("ERROR", data_stream.str());
            }
        }

        // 系统资源监测
        if (resource_info_valid_)
        {
            // 检查CPU使用率
            if (system_cpu_usage_ > 90.0)
            {
                std::ostringstream data_stream;
                data_stream << "HIGH_CPU_USAGE:" << system_cpu_usage_ << "%";
                LoggerInterface::logOperation("ERROR", data_stream.str());
            }

            // 检查内存使用率
            if (system_memory_usage_ > 90.0)
            {
                std::ostringstream data_stream;
                data_stream << "HIGH_MEMORY_USAGE:" << system_memory_usage_ << "%";
                LoggerInterface::logOperation("ERROR", data_stream.str());
            }

            // 检查磁盘使用率
            if (system_disk_usage_ > 90.0)
            {
                std::ostringstream data_stream;
                data_stream << "HIGH_DISK_USAGE:" << system_disk_usage_ << "%";
                LoggerInterface::logOperation("ERROR", data_stream.str());
            }
        }
    }

    // 修改statusCallback函数，只用于发送状态数据，不进行数据获取
    void statusCallback(const ros::TimerEvent &)
    {
        // 记录开始处理时间，用于性能分析
        auto start_time = ros::Time::now();

        try
        {
            // 更新ROS节点状态信息
            ROS_DEBUG("Starting ROS node status update");

            // 添加超时机制，确保即使updateNodeStatusInfo卡住，也能继续执行
            bool node_info_updated = false;
            std::thread node_info_thread([this, &node_info_updated]()
                                         {
                try {
                    updateNodeStatusInfo();
                    node_info_updated = true;
                } catch (const std::exception& e) {
                    ROS_ERROR("Node status update thread exception: %s", e.what());
                } });

            // 设置超时（2秒）
            if (node_info_thread.joinable())
            {
                std::chrono::milliseconds timeout(2000);
                auto future = std::async(std::launch::async, [&node_info_thread]()
                                         { node_info_thread.join(); });

                if (future.wait_for(timeout) == std::future_status::timeout)
                {
                    ROS_ERROR("Node status update timed out!");
                    // 无法正常join线程，让它自己在后台结束
                    node_info_thread.detach();
                }
            }

            if (!node_info_updated)
            {
                ROS_WARN("Node status update failed, using previous data");
            }

            // 移除固定电流值，使用订阅的数据
            battery_.remaining_percentage = 85.0;

            // 构建状态消息
            ROS_DEBUG("Building status message");
            std::ostringstream status_msg;
            status_msg << "STATUS:";

            // 添加电机数据
            status_msg << "MOTOR;";
            for (int i = 0; i < MOTOR_COUNT; i++)
            {
                status_msg << "M" << (i + 1) << ",";
                if (motors_[i].is_valid)
                {
                    status_msg << "pos:" << std::fixed << std::setprecision(6) << motors_[i].position << ","
                               << "vel:" << std::fixed << std::setprecision(6) << motors_[i].velocity << ","
                               << "tor:" << std::fixed << std::setprecision(6) << motors_[i].torque;
                }
                else
                {
                    status_msg << "pos:NULL,vel:NULL,tor:NULL";
                }
                if (i < MOTOR_COUNT - 1)
                    status_msg << ";";
            }

            // 添加IMU数据
            status_msg << ";IMU,";
            if (imu_.is_valid)
            {
                status_msg << "acc:" << imu_.acceleration.x << "/"
                           << imu_.acceleration.y << "/"
                           << imu_.acceleration.z << ","
                           << "gyro:" << imu_.angular_velocity.x << "/"
                           << imu_.angular_velocity.y << "/"
                           << imu_.angular_velocity.z << ","
                           << "euler:" << imu_.euler_angles.roll << "/"
                           << imu_.euler_angles.pitch << "/"
                           << imu_.euler_angles.yaw;
            }
            else
            {
                status_msg << "acc:NULL/NULL/NULL,"
                           << "gyro:NULL/NULL/NULL,"
                           << "euler:NULL/NULL/NULL";
            }

            // 添加电源状态
            status_msg << ";POWER,";
            if (power_.is_valid)
            {
                status_msg << "sys:" << (power_.system_power == PowerStatus::SwitchState::ON ? "1" : power_.system_power == PowerStatus::SwitchState::OFF ? "0"
                                                                                                                                                          : "NULL")
                           << ","
                           << "motor:" << (power_.motor_power == PowerStatus::SwitchState::ON ? "1" : power_.motor_power == PowerStatus::SwitchState::OFF ? "0"
                                                                                                                                                          : "NULL");
            }
            else
            {
                status_msg << "sys:NULL,motor:NULL";
            }

            // 添加电池状态
            status_msg << ";BATTERY,"
                       << "volt:" << (battery_.is_valid ? std::to_string(battery_.voltage) : "NULL") << ","
                       << "curr:" << (battery_.is_valid ? std::to_string(battery_.current) : "NULL") << ","
                       << "temp:" << (battery_.is_valid ? std::to_string(battery_.temperature) : "NULL") << ","
                       << "remain:" << (battery_.is_valid ? std::to_string(battery_.remaining_percentage) : "NULL");

            // 添加系统资源使用信息
            status_msg << ";SYSTEM_RESOURCES,";
            if (resource_info_valid_)
            {
                status_msg << "cpu:" << std::fixed << std::setprecision(2) << system_cpu_usage_ << "%,"
                           << "mem:" << std::fixed << std::setprecision(2) << system_memory_usage_ << "%,"
                           << "disk:" << std::fixed << std::setprecision(2) << system_disk_usage_ << "%,"
                           << "cpu_temp:" << std::fixed << std::setprecision(2) << cpu_temperature_ << "°C";
            }
            else
            {
                status_msg << "cpu:NULL,mem:NULL,disk:NULL,cpu_temp:NULL";
            }

            // 直接添加ROS节点状态信息到同一个消息中，而不是分开发送
            for (size_t i = 0; i < node_statuses_.size(); i++)
            {
                const auto &node = node_statuses_[i];
                status_msg << ";ROS_NODE,"
                           << "node:" << node.name << ","
                           << "status:" << node.status << ","
                           << "cpu:" << node.cpu << ","
                           << "memory:" << node.memory;
            }

            // 发送合并后的状态消息
            ROS_DEBUG("Sending status message");
            bool sent = sendMessage(status_msg.str());
            if (!sent)
            {
                ROS_ERROR("Failed to send status message");
            }

            // 计算并打印处理时间，用于性能分析
            auto end_time = ros::Time::now();
            double processing_time = (end_time - start_time).toSec() * 1000.0; // 转为毫秒
            ROS_DEBUG("Status callback processing time: %.2f ms", processing_time);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Status callback exception: %s", e.what());
        }
    }

    void heartbeatCallback(const ros::TimerEvent &)
    {
        sendMessage("HEARTBEAT");
    }

    void operationCallback(const livelybot_logger::LoggerOperation::ConstPtr &msg)
    {
        std::string message = "OPERATION:" + msg->operation_type + " | " + msg->operation_data;
        sendMessage(message);
    }

    // 获取系统资源占用信息
    void updateSystemResourceInfo()
    {
        try
        {
            // 获取CPU负载
            std::string cpu_cmd = "top -bn1 | grep 'Cpu(s)' | awk '{print $2 + $4}'";
            std::string cpu_output = execSystemCommandWithTimeout(cpu_cmd.c_str(), 500);
            if (!cpu_output.empty())
            {
                system_cpu_usage_ = std::stof(cpu_output);
            }

            // 获取内存使用率
            std::string mem_cmd = "free | grep Mem | awk '{print $3/$2 * 100.0}'";
            std::string mem_output = execSystemCommandWithTimeout(mem_cmd.c_str(), 500);
            if (!mem_output.empty())
            {
                system_memory_usage_ = std::stof(mem_output);
            }

            // 获取磁盘使用率
            std::string disk_cmd = "df -h / | grep / | awk '{print $5}' | sed 's/%//'";
            std::string disk_output = execSystemCommandWithTimeout(disk_cmd.c_str(), 500);
            if (!disk_output.empty())
            {
                system_disk_usage_ = std::stof(disk_output);
            }

            // 获取CPU温度
            try
            {
                // 方法1: 通过/sys/class/thermal获取温度
                std::string thermal_cmd = "cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -nr | head -n1";
                std::string thermal_output = execSystemCommandWithTimeout(thermal_cmd.c_str(), 500);
                if (!thermal_output.empty())
                {
                    // 转换为摄氏度（文件中通常是毫摄氏度）
                    cpu_temperature_ = std::stof(thermal_output) / 1000.0f;
                }
                else
                {
                    // 方法2: 通过sensors命令
                    std::string sensors_cmd = "sensors 2>/dev/null | grep -i 'core\\|temp' | grep ':' | awk '{print $2}' | sed 's/[^0-9.]//g' | sort -nr | head -n1";
                    std::string sensors_output = execSystemCommandWithTimeout(sensors_cmd.c_str(), 500);
                    if (!sensors_output.empty())
                    {
                        cpu_temperature_ = std::stof(sensors_output);
                    }
                    else
                    {
                        // 如果两种方法都失败，标记为无效数据
                        cpu_temperature_ = 0.0f;
                    }
                }
            }
            catch (const std::exception &e)
            {
                cpu_temperature_ = 0.0f;
                ROS_DEBUG("获取CPU温度失败: %s", e.what()); // 防止未使用的变量警告
            }

            resource_info_valid_ = true;
        }
        catch (const std::exception &e)
        {
            resource_info_valid_ = false;
            ROS_ERROR("Failed to get system resource info: %s", e.what());
        }
    }

    // 获取ROS节点状态信息
    void updateNodeStatusInfo()
    {
        // 预先定义要监控的节点列表 - 移到函数顶部，使其在try和catch块中都可访问
        std::vector<std::string> target_nodes = {
            "/joy_node",
            "/joy_teleop",
            "/livelybot_oled_node",
            "/power_node",
            "/rosout",
            "/livelybot_logger",
            "/sim2real_master_node",
            "/yesense_imu_node"};

        try
        {
            // 获取CPU核心数，用于标准化CPU使用率
            int cpu_cores = sysconf(_SC_NPROCESSORS_ONLN);
            if (cpu_cores <= 0)
                cpu_cores = 1; // 防止除零错误

            // 先获取运行中的所有ROS节点
            std::string node_list_cmd = "rosnode list";
            std::string node_list_output = execSystemCommandWithTimeout(node_list_cmd.c_str(), 1000);

            if (node_list_output.empty())
            {
                ROS_WARN("Unable to get rosnode list, skipping node status update");
                return;
            }

            // 清空之前的节点状态
            node_statuses_.clear();

            // 获取所有进程信息，一次性获取所有进程信息避免多次调用ps
            std::string all_procs_cmd = "ps aux | grep ros";
            std::string all_procs_output = execSystemCommandWithTimeout(all_procs_cmd.c_str(), 1000);

            // 临时映射存储进程名到CPU/内存使用率的映射
            std::map<std::string, std::pair<float, float>> process_resources;

            // 解析ps的输出获取每个ros相关进程的资源使用情况
            std::istringstream proc_stream(all_procs_output);
            std::string line;
            while (std::getline(proc_stream, line))
            {
                std::istringstream line_stream(line);
                std::string user, pid, cpu, mem, vsz, rss, tty, stat, start, time, command;

                if (!(line_stream >> user >> pid >> cpu >> mem))
                {
                    continue; // 无法解析这一行，跳过
                }

                // 跳过中间字段直到命令
                for (int i = 0; i < 6; i++)
                {
                    line_stream >> command;
                }

                // 获取完整命令行
                std::string cmd_line;
                std::getline(line_stream, cmd_line);

                // 尝试从命令行中提取节点名称
                for (const auto &node_name : target_nodes)
                {
                    std::string basename = node_name;
                    if (basename[0] == '/')
                    {
                        basename = basename.substr(1);
                    }

                    // 如果命令行包含节点名，记录其资源使用情况
                    if (cmd_line.find(basename) != std::string::npos)
                    {
                        try
                        {
                            float cpu_val = std::stof(cpu);
                            float mem_val = std::stof(mem);

                            // 标准化CPU使用率为单核百分比
                            cpu_val = cpu_val / cpu_cores;

                            process_resources[node_name] = {cpu_val, mem_val};
                            break;
                        }
                        catch (...)
                        {
                            // 转换失败，忽略
                        }
                    }
                }
            }

            // 检查每个节点是否运行并获取资源使用情况
            for (const auto &node_name : target_nodes)
            {
                std::string status = "stopped";
                std::string cpu_usage = "NULL";
                std::string memory_usage = "NULL";

                // 检查节点是否在运行列表中
                if (node_list_output.find(node_name) != std::string::npos)
                {
                    status = "run";

                    // 获取该节点的CPU和内存使用情况
                    auto it = process_resources.find(node_name);
                    if (it != process_resources.end())
                    {
                        std::ostringstream cpu_stream, mem_stream;
                        cpu_stream << std::fixed << std::setprecision(1) << it->second.first << "%";
                        mem_stream << std::fixed << std::setprecision(1) << it->second.second << "%";
                        cpu_usage = cpu_stream.str();
                        memory_usage = mem_stream.str();
                    }
                }

                // 添加到节点状态列表
                node_statuses_.emplace_back(node_name, status, cpu_usage, memory_usage);
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Failed to get node status info: %s", e.what());

            // 如果获取失败，使用默认值（NULL值）
            node_statuses_.clear();
            for (const auto &node_name : target_nodes)
            {
                node_statuses_.emplace_back(node_name, "unknown", "NULL", "NULL");
            }
        }
    }

public:
    LoggerNode() : nh_("~"), system_cpu_usage_(0.0f), system_memory_usage_(0.0f),
                   system_disk_usage_(0.0f), cpu_temperature_(0.0f), resource_info_valid_(false)
    {
        // 初始化节点状态信息，使用默认值而不是实时获取，避免在构造函数中卡住
        for (const auto &node_name : {"/joy_node", "/joy_teleop", "/livelybot_oled_node", "/power_node",
                                      "/rosout", "/livelybot_logger", "/sim2real_master_node", "/yesense_imu_node"})
        {
            node_statuses_.emplace_back(node_name, "unknown", "0%", "0%");
        }

        // 获取参数
        nh_.param("heartbeat_interval", heartbeat_interval_, 1.0);
        nh_.param("power_timeout", power_timeout_, 1.0);                         // 默认1秒超时
        nh_.param("data_acquisition_interval", data_acquisition_interval_, 0.1); // 默认100ms获取一次数据
        nh_.param("status_send_interval", status_send_interval_, 5.0);           // 默认5秒发送一次完整状态

        // 初始化socket
        initializeSocket();

        // 确保电源状态初始化为无效
        power_.is_valid = false;
        power_.system_power = PowerStatus::SwitchState::UNKNOWN;
        power_.motor_power = PowerStatus::SwitchState::UNKNOWN;

        // 发送启动日志
        sendMessage("OPERATION:livelybot_logger_node_start");

        // 订阅电机数据
        joint_state_sub_ = nh_.subscribe("/error_joint_states", 10, &LoggerNode::jointStateCallback, this);

        // 订阅IMU数据
        imu_sub_ = nh_.subscribe("/imu/data", 10, &LoggerNode::imuCallback, this);

        // 订阅功率板开关状态
        power_switch_sub_ = nh_.subscribe("/power_switch_state", 10, &LoggerNode::powerSwitchCallback, this);

        //订阅bms_error
        bms_error_sub_ = nh_.subscribe("/bms_error", 1, &LoggerNode::bmsErrorCallback, this);

        // 添加电池电压订阅
        battery_volt_sub_ = nh_.subscribe("/battery_voltage", 1, &LoggerNode::batteryVoltCallback, this);

        // 添加电池电流订阅
        battery_curr_sub_ = nh_.subscribe("/battery_current", 1, &LoggerNode::batteryCurrCallback, this);

        // 添加电池温度订阅
        battery_temp_sub_ = nh_.subscribe("/battery_temperature", 1, &LoggerNode::batteryTempCallback, this);

        // 创建电源状态超时检测定时器
        power_timeout_timer_ = nh_.createTimer(ros::Duration(0.1), &LoggerNode::powerTimeoutCallback, this);

        // 创建快速数据获取定时器
        data_acquisition_timer_ = nh_.createTimer(ros::Duration(data_acquisition_interval_),
                                                  &LoggerNode::dataAcquisitionCallback, this);

        // 先发送一次状态数据
        statusCallback(ros::TimerEvent());

        // 创建定时器，5秒发送一次状态数据
        status_timer_ = nh_.createTimer(ros::Duration(status_send_interval_),
                                        &LoggerNode::statusCallback, this);

        // 创建心跳定时器，1秒发送一次
        heartbeat_timer_ = nh_.createTimer(ros::Duration(heartbeat_interval_),
                                           &LoggerNode::heartbeatCallback, this);

        operation_sub_ = nh_.subscribe("/logger/operation", 10,
                                       &LoggerNode::operationCallback, this);

        ROS_INFO("Sim2Real Logger running with data acquisition interval: %.2f sec, status send interval: %.2f sec!",
                 data_acquisition_interval_, status_send_interval_);
    }

    ~LoggerNode()
    {
        if (socket_fd_ >= 0)
        {
            close(socket_fd_);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livelybot_logger");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 先初始化LoggerInterface
    livelybot_logger::LoggerInterface::init(nh, "livelybot_logger");

    // 创建LoggerNode实例
    LoggerNode logger_node;

    // livelybot_logger::LoggerInterface::logOperation("MODE_SWITCH", "Entering default mode");

    ros::spin();
    return 0;
}
