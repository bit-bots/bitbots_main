#include <rclcpp/rclcpp.hpp>
#include "oled_interface.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "livelybot_oled_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    oled_mission(node);

    rclcpp::shutdown();
    return 0;
}
