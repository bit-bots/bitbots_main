#include <rclcpp/rclcpp.hpp>
#include "hardware/robot.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // automatically_declare_parameters_from_overrides allows parameters loaded
    // from a YAML param file (--ros-args --params-file robot.yaml) to be
    // accessible via get_parameter() without prior declare_parameter() calls.
    auto node = std::make_shared<rclcpp::Node>(
        "robot_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    livelybot_serial::robot robot(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
