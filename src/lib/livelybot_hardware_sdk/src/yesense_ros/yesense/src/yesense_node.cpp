#include <rclcpp/rclcpp.hpp>
#include "yesense_driver.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("yesense_imu");
    yesense::YesenseDriver driver(node);
    driver.run();
    rclcpp::shutdown();
    return 0;
}
