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
    ros::init(argc, argv, "test_timeout");
    ros::NodeHandle n;
    ros::Rate r(100);
    livelybot_serial::robot rb;
    //rb.set_motor_runzero();     // 电机上电自动回零
    const int motor_num = rb.Motors.size();
    ROS_INFO("ALL Motor: %d\n", motor_num);
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    int cont = 0;
    float angle = 0.2;

    /* 逐个设置电机的超时时间，在 robot 的构造函数中有 set_motorout 函数可以对设置所有电机的超时时间 */ 
    for (int i = 0; i < 5; i++)
    {
        for (motor *m : rb.Motors)
        {
            m->set_motorout(10000);
        }
        rb.motor_send_2();
        ros::Duration(0.01).sleep();
    }
    

    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        /////////////////////////send
        for (int i = 0; i < motor_num; i++)
        {
            rb.Motors[i]->velocity(0.3);
        }
        rb.motor_send_2();


        ////////////////////////recv
        for (motor *m : rb.Motors)
        {
            motor_back_t motor;
            motor = *m->get_current_motor_state();
            ROS_INFO("ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.torque);
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

