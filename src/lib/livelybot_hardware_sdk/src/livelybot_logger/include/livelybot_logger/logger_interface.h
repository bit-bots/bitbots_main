#ifndef LIVELYBOT_LOGGER_INTERFACE_H
#define LIVELYBOT_LOGGER_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include "livelybot_logger/msg/logger_operation.hpp"

namespace livelybot_logger
{

/**
 * Singleton utility class that lets any node publish a LoggerOperation message
 * to the /logger/operation topic without owning the ROS 2 node directly.
 *
 * Usage:
 *   LoggerInterface::init(node, "my_node");
 *   LoggerInterface::logOperation("MODE_SWITCH", "entering walk");
 */
class LoggerInterface
{
private:
    static rclcpp::Publisher<livelybot_logger::msg::LoggerOperation>::SharedPtr operation_pub_;
    static std::string node_name_;
    static bool initialized_;
    static rclcpp::Node::SharedPtr node_;

    LoggerInterface() = default;

public:
    static LoggerInterface &getInstance();

    /** Must be called once before logOperation(). */
    static void init(rclcpp::Node::SharedPtr node, const std::string &node_name);

    /**
     * Publish a LoggerOperation message.
     * @param operation_type  Short category string, e.g. "MODE_SWITCH" or "ERROR".
     * @param operation_data  Human-readable detail string.
     * @param result          Outcome string, defaults to "success".
     */
    static void logOperation(const std::string &operation_type,
                             const std::string &operation_data,
                             const std::string &result = "success");
};

}  // namespace livelybot_logger

#endif  // LIVELYBOT_LOGGER_INTERFACE_H
