#include <rclcpp/rclcpp.hpp>
#include "hardware/robot.h"
#include <thread>
#include <chrono>
#include <cstdio>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "motor_set_zero",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    livelybot_serial::robot rb(node);
    RCLCPP_INFO(node->get_logger(), "\033[1;32mSTART\033[0m");

    rb.set_reset_zero();  // reset zero position for all motors

    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    rclcpp::Rate rate(100);
    while (rclcpp::ok())
    {
        int num = 0;
        for (motor *m : rb.Motors)
        {
            auto *s = m->get_current_motor_state();
            printf("Motors[%02d]: pos %.4f  vel %.4f  tor %.4f\n",
                   num++, s->position, s->velocity, s->torque);
            rb.send_get_motor_state_cmd();
        }
        rb.motor_send_2();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
