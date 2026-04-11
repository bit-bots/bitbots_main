#include <rclcpp/rclcpp.hpp>
#include "hardware/robot.h"
#include <iostream>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "motor_move_zero",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    livelybot_serial::robot rb(node);
    RCLCPP_INFO(node->get_logger(), "\033[1;32mSTART\033[0m");

    rclcpp::Rate rate(400);
    int i = 0;
    while (rclcpp::ok())
    {
        rb.detect_motor_limit();
        i = 0;
        for (motor *m : rb.Motors)
        {
            i++;
            RCLCPP_INFO(node->get_logger(), "motor id:%d  pos:%.2f",
                        i, m->get_current_motor_state()->position);
            m->pos_vel_MAXtqe(0.0f, 0.3f, 100.0f);
        }
        rb.motor_send_2();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
