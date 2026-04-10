#include "livelybot_logger/logger_interface.h"

namespace livelybot_logger
{

    // 静态成员的定义
    ros::Publisher LoggerInterface::operation_pub_;
    std::string LoggerInterface::node_name_;
    bool LoggerInterface::initialized_ = false;
    // 私有化构造函数的实现
    LoggerInterface::LoggerInterface()
    {
        // 这里可以进行其他初始化操作
    }

    LoggerInterface &LoggerInterface::getInstance()
    {
        static LoggerInterface instance;
        return instance;
    }

    void LoggerInterface::init(ros::NodeHandle &nh, const std::string &node_name)
    {
        if (initialized_)
        {
            return;
        }
        node_name_ = node_name;
        operation_pub_ = nh.advertise<livelybot_logger::LoggerOperation>("/logger/operation", 10);
        initialized_ = true;
        ROS_INFO("LoggerInterface initialized with node: %s", node_name.c_str());
    }

    void LoggerInterface::logOperation(const std::string &operation_type, const std::string &operation_data, const std::string &result)
    {
        if (!initialized_)
        {
            ros::NodeHandle nh;
            init(nh, "default_logger_node");
        }

        livelybot_logger::LoggerOperation msg;
        msg.timestamp = ros::Time::now();
        msg.node_name = node_name_;
        msg.operation_type = operation_type;
        msg.operation_data = operation_data;
        msg.result = result;
        operation_pub_.publish(msg);
    }

}