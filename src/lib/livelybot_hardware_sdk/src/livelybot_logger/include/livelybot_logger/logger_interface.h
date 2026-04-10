#ifndef livelybot_logger_INTERFACE_H
#define livelybot_logger_INTERFACE_H

#include <ros/ros.h>
#include "livelybot_logger/LoggerOperation.h"

namespace livelybot_logger
{
    class LoggerInterface
    {
    private:
        static ros::Publisher operation_pub_; // 静态成员声明
        static std::string node_name_;
        static bool initialized_;

        // 私有化构造函数，防止外部实例化
        LoggerInterface();

    public:
        // 获取单例对象的接口
        static LoggerInterface &getInstance();

        // 初始化函数
        static void init(ros::NodeHandle &nh, const std::string &node_name);

        // 日志记录函数
        static void logOperation(const std::string &operation_type, const std::string &operation_data, const std::string &result = "success");
    };

} // namespace livelybot_logger

#endif // livelybot_logger_INTERFACE_H
