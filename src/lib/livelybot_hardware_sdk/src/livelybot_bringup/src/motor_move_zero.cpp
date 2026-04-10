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
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle n;
    ros::Rate r(400);
    livelybot_serial::robot rb;
    //rb.set_motor_runzero();     // 电机上电自动回零
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================

    int i = 0;
    // ros::Duration(5).sleep();
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        /////////////////////////send
        rb.detect_motor_limit();
        i = 0;
        for (motor *m : rb.Motors)
        {   
            i++;
            ROS_INFO("motor id:%d, pos %.2f", i, m->get_current_motor_state()->position);
            // ROS_INFO("id %d pos %f vel %f tqe %f", m->get_current_motor_state()->ID, m->get_current_motor_state()->position, m->get_current_motor_state()->velocity, m->get_current_motor_state()->torque);
            // m->fresh_cmd_int16(angle, 0.05, 1, 200.0, 0, 5, 0.1, 0, 0.5);
            m->pos_vel_MAXtqe(0, 0.3, 100);
        }
        rb.motor_send_2();

        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO_STREAM("END"); 
    return 0;
}
