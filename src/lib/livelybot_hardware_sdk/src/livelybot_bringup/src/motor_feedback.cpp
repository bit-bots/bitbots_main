#include <rclcpp/rclcpp.hpp>
#include "hardware/robot.h"
#include <iostream>
#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "motor_feedback",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    livelybot_serial::robot rb(node);
    RCLCPP_INFO(node->get_logger(), "\033[1;32mSTART\033[0m");
    RCLCPP_INFO(node->get_logger(), "Motor count: %ld", rb.Motors.size());

    rb.send_get_motor_state_cmd();

    rclcpp::Rate rate(300);
    while (rclcpp::ok())
    {
        rb.detect_motor_limit();
        for (motor *m : rb.Motors)
        {
            auto *s = m->get_current_motor_state();
            RCLCPP_INFO(node->get_logger(), "motor:%d  pos:%.4f  vel:%.4f  tor:%.4f",
                        s->ID, s->position, s->velocity, s->torque);
        }
        rb.motor_send_2();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
