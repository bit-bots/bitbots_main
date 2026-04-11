#include <rclcpp/rclcpp.hpp>
#include "hardware/robot.h"
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "motor_timeout",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    livelybot_serial::robot rb(node);
    const int motor_num = static_cast<int>(rb.Motors.size());
    RCLCPP_INFO(node->get_logger(), "All motors: %d", motor_num);
    RCLCPP_INFO(node->get_logger(), "\033[1;32mSTART\033[0m");

    // Set per-motor timeout individually
    for (int i = 0; i < 5; i++)
    {
        for (motor *m : rb.Motors)
        {
            m->set_motorout(10000);
        }
        rb.motor_send_2();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rclcpp::Rate rate(100);
    while (rclcpp::ok())
    {
        for (int idx = 0; idx < motor_num; idx++)
        {
            rb.Motors[idx]->velocity(0.3f);
        }
        rb.motor_send_2();

        for (motor *m : rb.Motors)
        {
            motor_back_t s = *m->get_current_motor_state();
            RCLCPP_INFO(node->get_logger(),
                        "ID:%d  pos:%8.4f  vel:%8.4f  tor:%8.4f",
                        s.ID, s.position, s.velocity, s.torque);
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
