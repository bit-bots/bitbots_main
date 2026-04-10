#include "ros/ros.h"
#ifndef RELEASE
#include "robot.h"
#else
#include "livelybot_serial/hardware/robot.h"
#endif
#include <iostream>
#include <thread>
#include <condition_variable>
#include "std_msgs/Float32MultiArray.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor_feedback");
    ros::NodeHandle n;
    ros::Rate r(300);
    livelybot_serial::robot rb;
    //rb.set_motor_runzero();     // 电机上电自动回零
    ROS_INFO("\033[1;32mSTART\033[0m");


    // ========================== singlethread send ===================== 
    ROS_INFO("motor num %ld" ,rb.Motors.size());

    rb.send_get_motor_state_cmd();
    while (ros::ok())
    {
        rb.detect_motor_limit();
        for (motor *m : rb.Motors)
        {
            auto motor_state = m->get_current_motor_state();
            ROS_INFO("motor:%d, position:%f, velocity:%f, torque:%f", motor_state->ID, motor_state->position, motor_state->velocity, motor_state->torque);
        }
        rb.motor_send_2();
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}