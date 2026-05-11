#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "sensor_actuator_status.hpp"
#include "ip_addr.hpp"
#include "oled_interface.hpp"

static Sensor_actuator_status *g_status = nullptr;
static uint8_t g_imu_flag = 0;
static float   g_rpy[3]   = {0.0f, 0.0f, 0.0f};
static int     g_motor_num[4] = {0, 0, 0, 0};

static void imu_callback(const sensor_msgs::msg::Imu::SharedPtr data)
{
    tf2::Quaternion q(data->orientation.x, data->orientation.y,
                      data->orientation.z, data->orientation.w);
    tf2::Matrix3x3 rot(q);
    double roll, pitch, yaw;
    rot.getRPY(roll, pitch, yaw);

    g_rpy[0] = static_cast<float>(roll);
    g_rpy[1] = static_cast<float>(pitch);
    g_rpy[2] = static_cast<float>(yaw);

    if (g_status) {
        g_status->send_imu_status(true, g_rpy);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    g_imu_flag = 0;
}

static void motor_callback(const sensor_msgs::msg::JointState::SharedPtr data)
{
    const int total = g_motor_num[0] + g_motor_num[1] + g_motor_num[2] + g_motor_num[3];
    unsigned char motor_status[64] = {};
    for (int i = 0; i < total; i++) {
        motor_status[i] = (data->position[static_cast<size_t>(i)] < -900.0) ? 0 : 1;
    }
    if (g_status) {
        g_status->send_motor_status(motor_status);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

static void battery_voltage_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    if (g_status) {
        g_status->send_battery_volt(msg->data);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

static void fsm_state_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (g_status) {
        g_status->send_fsm_state(msg->data);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void oled_mission(rclcpp::Node::SharedPtr node)
{
    // Read CAN port and motor count from robot params
    int can_port_num = 0;
    if (!node->get_parameter("robot.CANboard.No_1_CANboard.CANport_num", can_port_num)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get robot.CANboard.No_1_CANboard.CANport_num");
    } else {
        RCLCPP_INFO(node->get_logger(), "Robot has %d CAN port(s)", can_port_num);
        for (int i = 0; i < can_port_num && i < 4; i++) {
            const std::string key = "robot.CANboard.No_1_CANboard.CANport.CANport_" +
                                    std::to_string(i + 1) + ".motor_num";
            if (!node->get_parameter(key, g_motor_num[i])) {
                RCLCPP_ERROR(node->get_logger(), "Failed to get %s", key.c_str());
            } else {
                RCLCPP_INFO(node->get_logger(), "CAN%d has %d motor(s)", i + 1, g_motor_num[i]);
            }
        }
    }
    RCLCPP_INFO(node->get_logger(), "Total motors: %d",
                g_motor_num[0] + g_motor_num[1] + g_motor_num[2] + g_motor_num[3]);

    g_status = new Sensor_actuator_status(g_motor_num[0], g_motor_num[1],
                                          g_motor_num[2], g_motor_num[3]);

    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 1, imu_callback);
    auto motor_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/error_joint_states", 1, motor_callback);
    auto battery_sub = node->create_subscription<std_msgs::msg::Float32>(
        "/battery_voltage", 1, battery_voltage_callback);
    auto fsm_sub = node->create_subscription<std_msgs::msg::Int32>(
        "/fsm_state", 1, fsm_state_callback);

    int ip_counter = 0;
    update_ip_addr();

    rclcpp::Rate rate(10.0);
    while (rclcpp::ok()) {
        rate.sleep();
        rclcpp::spin_some(node);

        if (++ip_counter >= 10) {
            ip_counter = 0;
            update_ip_addr();
        }

        g_status->send_ip_addr(get_ip_data_u32_all(), 3);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (g_imu_flag) {
            g_status->send_imu_status(false, g_rpy);
        }
        g_imu_flag = 1;
    }

    delete g_status;
    g_status = nullptr;
}
