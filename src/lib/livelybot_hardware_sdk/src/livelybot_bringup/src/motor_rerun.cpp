#include <rclcpp/rclcpp.hpp>
#include "hardware/robot.h"
#include <iostream>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "motor_rerun",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    livelybot_serial::robot rb(node);
    const int motor_num = static_cast<int>(rb.Motors.size());
    RCLCPP_INFO(node->get_logger(), "All motors: %d", motor_num);
    RCLCPP_INFO(node->get_logger(), "\033[1;32mSTART\033[0m");

    rclcpp::Rate rate(200);
    int cont = 0;
    float angle = 0.2f;

    while (rclcpp::ok())
    {
        for (int idx = 0; idx < motor_num; idx++)
        {
            if (idx < motor_num / 2)
                rb.Motors[idx]->pos_vel_MAXtqe(angle, 0.1f, 10.0f);
            else
                rb.Motors[idx]->pos_vel_MAXtqe(-angle, 0.1f, 10.0f);
        }
        if (++cont >= 250)
        {
            cont = 0;
            angle *= -1;
        }
        rb.motor_send_2();

        for (motor *m : rb.Motors)
        {
            motor_back_t s = *m->get_current_motor_state();
            RCLCPP_INFO(node->get_logger(),
                        "ID:%2d mode:%2d fault:%2X pos:%8.4f vel:%8.4f tor:%8.4f",
                        s.ID, s.mode, s.fault, s.position, s.velocity, s.torque);
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
