#include "livelybot_logger/logger_interface.h"

namespace livelybot_logger
{

// Static member definitions
rclcpp::Publisher<livelybot_logger::msg::LoggerOperation>::SharedPtr LoggerInterface::operation_pub_;
std::string LoggerInterface::node_name_;
bool LoggerInterface::initialized_ = false;
rclcpp::Node::SharedPtr LoggerInterface::node_;

LoggerInterface &LoggerInterface::getInstance()
{
    static LoggerInterface instance;
    return instance;
}

void LoggerInterface::init(rclcpp::Node::SharedPtr node, const std::string &node_name)
{
    if (initialized_) {
        return;
    }
    node_ = node;
    node_name_ = node_name;
    operation_pub_ = node_->create_publisher<livelybot_logger::msg::LoggerOperation>("/logger/operation", 10);
    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "LoggerInterface initialized for node: %s", node_name.c_str());
}

void LoggerInterface::logOperation(const std::string &operation_type,
                                   const std::string &operation_data,
                                   const std::string &result)
{
    if (!initialized_ || !operation_pub_) {
        return;
    }

    livelybot_logger::msg::LoggerOperation msg;
    msg.timestamp = node_->now();
    msg.node_name = node_name_;
    msg.operation_type = operation_type;
    msg.operation_data = operation_data;
    msg.result = result;
    operation_pub_->publish(msg);
}

}  // namespace livelybot_logger
