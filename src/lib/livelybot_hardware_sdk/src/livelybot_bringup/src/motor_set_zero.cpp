#include "ros/ros.h"
#ifndef RELEASE
#include "robot.h"
#else
#include "livelybot_serial/hardware/robot.h"
#endif
#include <iostream>
#include <thread>
#include <condition_variable>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_reset_zero");
    ros::NodeHandle n;
    ros::Rate r(100);
    livelybot_serial::robot rb;

    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    rb.set_reset_zero();  // 全部电机重置零位
    // rb.set_reset_zero({0, 1, 3, 4});  // 指定电机重置零位，参数为 Motors 的下标

    ros::Duration(1.5).sleep();  // 延时，方便查看结果的，可去掉
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        int num = 0;
        for (motor *m : rb.Motors)
        {   
            printf("Motrs[%02d]: pos %f, vel %f, tqe %f\n", num++, m->get_current_motor_state()->position, m->get_current_motor_state()->velocity, m->get_current_motor_state()->torque);
            rb.send_get_motor_state_cmd();
        }
        rb.motor_send_2();
        
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
