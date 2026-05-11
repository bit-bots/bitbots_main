#include <rclcpp/rclcpp.hpp>
#include "hardware/robot.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "canboard_update",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    livelybot_serial::robot rb(node);
    rb.canboard_bootloader();

    rclcpp::shutdown();
    return 0;
}
