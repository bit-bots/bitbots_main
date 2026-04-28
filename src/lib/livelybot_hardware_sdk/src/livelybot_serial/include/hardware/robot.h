#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <iostream>
#include "canboard.h"
#include "motor.h"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <initializer_list>
#include <fstream>
#include <dirent.h>
#include <algorithm>
#include <atomic>
#include <sensor_msgs/msg/joint_state.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <bitbots_msgs/msg/joint_torque.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <mutex>
#include <unordered_map>
#include <string>
#include <cmath>
#include <functional>

namespace livelybot_serial
{

/**
 * @brief Top-level robot object that owns the hardware communication stack.
 *
 * On construction the robot reads its configuration from the ROS 2 parameter
 * server, opens the serial ports, constructs CAN boards / ports / motors,
 * and starts background threads for data receive, joint-state publishing and
 * serial error recovery.
 *
 * SDK version: 4.3.3
 */
class robot
{
private:
    std::string robot_name;
    std::string Serial_Type;
    int CANboard_num = 0;
    int Seial_baudrate = 0;
    int SDK_version = -1;
    rclcpp::Node::SharedPtr node_;
    std::vector<canboard> CANboards;
    std::vector<std::string> str;
    std::string SDK_version_str = "4.3.3";
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<bitbots_msgs::msg::JointCommand>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<bitbots_msgs::msg::JointTorque>::SharedPtr torque_sub_;
    std::unordered_set<std::string> torque_off_motors_;
    std::mutex cmd_mutex_;

    // --- diagnostics ---
    /** Per-motor state used by the diagnostic task to detect sustained faults. */
    struct MotorDiagState {
        double torque_high_since = 0.0;  /**< steady_clock seconds when overload started; 0 = not overloaded */
    };

    std::unique_ptr<diagnostic_updater::Updater>        diag_updater_;
    std::unordered_map<std::string, MotorDiagState>     diag_state_;

    double diag_connection_timeout_;
    double diag_torque_overload_threshold_;
    double diag_torque_overload_duration_;
    std::thread error_check_thread_;
    rclcpp::TimerBase::SharedPtr state_poll_timer_;
    fun_version fun_v = fun_v1;
    float slave_v = 3.0f;
    int control_type = 0;

    /** Read USB Vendor ID and Product ID from the Linux sysfs. */
    static bool read_usb_vid_pid(const std::string &device, int &vid, int &pid);

public:
    std::vector<lively_serial *>  ser;
    std::vector<motor *>          Motors;
    std::vector<canport *>        CANPorts;
    std::vector<std::thread>      ser_recv_threads;
    int motor_position_limit_flag = 0;
    int motor_torque_limit_flag   = 0;

    /**
     * @param node  Shared ROS 2 node; all parameters are loaded from its parameter server.
     */
    explicit robot(rclcpp::Node::SharedPtr node);
    ~robot();

    void publishJointStates();
    void detect_motor_limit();

    /** Receive a JointCommand and forward position/velocity goals to matching motors. */
    void jointCommandCallback(bitbots_msgs::msg::JointCommand::ConstSharedPtr msg);

    /** Populate one motor's DiagnosticStatus (called by the updater timer). */
    void motorDiagnostic(const std::string &name, motor *m,
                         diagnostic_updater::DiagnosticStatusWrapper &stat);

    /**
     * Enable or disable torque for individual motors.
     *
     * Mirrors the Dynamixel-era `set_torque_individual` API used by the HCM
     * and the animation recorder for puppeteering (teach mode):
     *   on=false — zero-torque command, motor becomes back-driveable.
     *   on=true  — re-enable; the next JointCommand will restore position control.
     */
    void torqueCallback(bitbots_msgs::msg::JointTorque::ConstSharedPtr msg);
    void motor_send_2();

    /** Classify a serial port by its USB VID/PID; returns number of ports per board, or <0 on mismatch. */
    int serial_pid_vid(const char *name, int *pid, int *vid);
    int serial_pid_vid(const char *name);

    /** List all device nodes under the given path prefix (e.g. "/dev/ttyACM"). */
    std::vector<std::string> list_serial_ports(const std::string &prefix);

    void init_ser();
    void check_error();
    int  check_serial_dev_exist(int max_index);
    void set_port_motor_num();
    void send_get_motor_state_cmd();
    void send_get_motor_version_cmd();
    void chevk_motor_connection_position();
    void chevk_motor_connection_version();
    void set_stop();
    void set_reset();
    void set_reset_zero();
    void set_reset_zero(std::initializer_list<int> motors);
    void set_motor_runzero();
    void set_timeout(int16_t t_ms);
    void set_timeout(uint8_t portx, int16_t t_ms);
    void motor_version_detection();
    void set_data_reset();
    void canboard_bootloader();
    void canboard_fdcan_reset();
};

} /* namespace livelybot_serial */

#endif /* _ROBOT_H_ */
