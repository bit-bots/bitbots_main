#pragma once

#include "livelybot_can_driver.hpp"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/BatteryState.h>
#include <livelybot_power/Power_switch.h>
#include <livelybot_power/Power_detect.h>

#define CAN_DEVICE_NAME "can0"

namespace livelybot_can
{
    class Power_Board
    {
    public:
        Power_Board();
        void run(ros::NodeHandle &n);
    private:
    };
}