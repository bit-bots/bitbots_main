#include <iostream>
#include <ros/ros.h>
#include "livelybot_power.hpp"
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h> // 添加 Int8 消息头文件
#include <livelybot_power/Power_switch.h>

// 全局变量
ros::Publisher battery_volt_pub;
ros::Publisher battery_curr_pub; // 添加电池电流发布者
ros::Publisher battery_temp_pub; // 添加电池温度发布者
ros::Publisher power_switch_pub; // 添加功率板开关状态发布者
ros::Publisher bms_error_pub;    // 添加 BMS 错误话题发布者

// 自定义CAN接收回调函数
void custom_can_recv_parse(int can_id, unsigned char *data, unsigned char len)
{
    uint8_t dev_addr;
    uint8_t data_type;
    uint8_t append_flag;

    dev_addr = can_id >> 7;
    data_type = (can_id >> 1) & 0x3F;
    append_flag = can_id >> 10;

// BMS地址定义
#define BMS_ADDR 0x06
// 功率板地址定义
#define POWER_SWITCH_ADDR 0x07

    if (!append_flag && dev_addr == BMS_ADDR && data_type == 0x01)
    {
        // 发布电池电压
        std_msgs::Float32 volt_msg;
        volt_msg.data = (*(int16_t *)&data[0]) / 100.0f;
        battery_volt_pub.publish(volt_msg);

        // 发布电池电流
        std_msgs::Float32 curr_msg;
        curr_msg.data = (*(int16_t *)&data[2]) / 100.0f;
        battery_curr_pub.publish(curr_msg);

        // 发布电池温度
        std_msgs::Float32 temp_msg;
        temp_msg.data = (*(int16_t *)&data[4]) / 100.0f;
        battery_temp_pub.publish(temp_msg);
    }
    else if (!append_flag && dev_addr == BMS_ADDR && data_type == 0x02)
    {
        // 发布 BMS 错误信息
        std_msgs::Int8 error_msg;
        error_msg.data = data[0];
        bms_error_pub.publish(error_msg);
    }
    else if (!append_flag && dev_addr == POWER_SWITCH_ADDR && data_type == 0x02)
    {
        // 处理功率板开关状态
        livelybot_power::Power_switch power_switch_msg;
        power_switch_msg.control_switch = data[0];
        power_switch_msg.power_switch = data[1];
        power_switch_pub.publish(power_switch_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "power_node");
    ros::NodeHandle n;

    // 初始化发布者
    battery_volt_pub = n.advertise<std_msgs::Float32>("battery_voltage", 1);
    battery_curr_pub = n.advertise<std_msgs::Float32>("battery_current", 1);
    battery_temp_pub = n.advertise<std_msgs::Float32>("battery_temperature", 1);            // 添加温度话题
    power_switch_pub = n.advertise<livelybot_power::Power_switch>("power_switch_state", 1); // 添加功率板开关状态发布
    bms_error_pub = n.advertise<std_msgs::Int8>("bms_error", 1); // 初始化 BMS 错误话题发布者

    // 初始化CAN驱动
    livelybot_can::CAN_Driver can_handler("can0");
    can_handler.start_callback(custom_can_recv_parse);

    // 主循环
    ros::Rate r(10);
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
