#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include "livelybot_msg/msg/power_switch.hpp"
#include "livelybot_msg/msg/power_detect.hpp"
#include "livelybot_can_driver.hpp"

#define CAN_DEVICE_NAME "can0"

#define BMS_ADDR          0x06
#define POWER_SWITCH_ADDR 0x07
#define ORANGEPI_ADDR     0x01

// Publishers are stored on the node so the CAN callback can reach them.
// Using a global pointer is the simplest pattern here because the CAN callback
// is a plain C function pointer with no user-data slot.
static rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr g_battery_volt_pub;
static rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr g_battery_curr_pub;
static rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr g_battery_temp_pub;
static rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr g_bms_error_pub;
static rclcpp::Publisher<livelybot_msg::msg::PowerSwitch>::SharedPtr g_power_switch_pub;
static rclcpp::Publisher<livelybot_msg::msg::PowerDetect>::SharedPtr g_power_detect_pub;
static livelybot_can::CAN_Driver *g_can_handler = nullptr;

/** Parse incoming CAN frames and publish ROS 2 messages. */
void can_recv_parse(int can_id, unsigned char *data, unsigned char /*dlc*/)
{
    const uint8_t dev_addr    = static_cast<uint8_t>(can_id >> 7);
    const uint8_t data_type   = static_cast<uint8_t>((can_id >> 1) & 0x3F);
    const uint8_t append_flag = static_cast<uint8_t>(can_id >> 10);

    if (append_flag) {
        return;
    }

    if (dev_addr == BMS_ADDR) {
        if (data_type == 0x01) {
            // Voltage, current, and temperature from BMS status frame
            std_msgs::msg::Float32 volt_msg;
            volt_msg.data = static_cast<float>(*reinterpret_cast<int16_t *>(&data[0])) / 100.0f;
            g_battery_volt_pub->publish(volt_msg);

            std_msgs::msg::Float32 curr_msg;
            curr_msg.data = static_cast<float>(*reinterpret_cast<int16_t *>(&data[2])) / 100.0f;
            g_battery_curr_pub->publish(curr_msg);

            std_msgs::msg::Float32 temp_msg;
            temp_msg.data = static_cast<float>(*reinterpret_cast<int16_t *>(&data[4])) / 100.0f;
            g_battery_temp_pub->publish(temp_msg);
        } else if (data_type == 0x02) {
            // BMS error code
            std_msgs::msg::Int8 error_msg;
            error_msg.data = static_cast<int8_t>(data[0]);
            g_bms_error_pub->publish(error_msg);
        }
    } else if (dev_addr == POWER_SWITCH_ADDR) {
        if (data_type == 0x01) {
            // Per-channel voltage and current from power switch board
            livelybot_msg::msg::PowerDetect detect_msg;
            detect_msg.voltage = static_cast<float>(*reinterpret_cast<int16_t *>(&data[0])) / 100.0f;
            detect_msg.current = static_cast<float>(*reinterpret_cast<int16_t *>(&data[2])) / 100.0f;
            detect_msg.power   = detect_msg.voltage * detect_msg.current;
            g_power_detect_pub->publish(detect_msg);
        } else if (data_type == 0x02) {
            // Switch status
            livelybot_msg::msg::PowerSwitch switch_msg;
            switch_msg.control_switch = data[0];
            switch_msg.power_switch   = data[1];
            g_power_switch_pub->publish(switch_msg);
        }
    }
}

/** Subscriber callback: forward switch commands to the power board via CAN. */
void power_switch_cmd_callback(const livelybot_msg::msg::PowerSwitch::SharedPtr msg)
{
    const uint32_t can_id = (ORANGEPI_ADDR << 7) | (1 << 1) | 0;
    uint8_t data[2] = {msg->control_switch, msg->power_switch};
    g_can_handler->send(can_id, data, 2);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("power_node");

    g_battery_volt_pub  = node->create_publisher<std_msgs::msg::Float32>("battery_voltage", 1);
    g_battery_curr_pub  = node->create_publisher<std_msgs::msg::Float32>("battery_current", 1);
    g_battery_temp_pub  = node->create_publisher<std_msgs::msg::Float32>("battery_temperature", 1);
    g_bms_error_pub     = node->create_publisher<std_msgs::msg::Int8>("bms_error", 1);
    g_power_switch_pub  = node->create_publisher<livelybot_msg::msg::PowerSwitch>("power_switch_state", 1);
    g_power_detect_pub  = node->create_publisher<livelybot_msg::msg::PowerDetect>("power_detect_state", 1);

    auto power_switch_sub = node->create_subscription<livelybot_msg::msg::PowerSwitch>(
        "power_switch_control", 1, power_switch_cmd_callback);

    livelybot_can::CAN_Driver can_handler(CAN_DEVICE_NAME);
    g_can_handler = &can_handler;
    can_handler.start_callback(can_recv_parse);

    RCLCPP_INFO(node->get_logger(), "Power node started, listening on %s", CAN_DEVICE_NAME);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
