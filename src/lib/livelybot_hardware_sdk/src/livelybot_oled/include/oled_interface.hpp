#pragma once

#include <rclcpp/rclcpp.hpp>

/** Start the OLED display mission loop. Blocks until the node shuts down. */
void oled_mission(rclcpp::Node::SharedPtr node);
