#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
// #include "robot_node.h"

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "joint_state_listener");
    ros::NodeHandle nh;
    

    // 创建一个订阅者对象，订阅名为"joint_states"的topic，队列长度设置为10
    livelybot_serial::robot_node rbn;
    

    // 进入ROS事件循环
    ros::spin();

    return 0;
}