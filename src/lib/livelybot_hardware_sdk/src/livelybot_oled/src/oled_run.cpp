#include <ros/ros.h>
#include "oled_interface.hpp"

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "oled_run");

    // 创建节点句柄
    ros::NodeHandle n;

    oled_mission(n);
}