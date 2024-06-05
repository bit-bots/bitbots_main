#ifndef BITBOTS_UTILS__UTILS_H_
#define BITBOTS_UTILS__UTILS_H_

#include <tf2_ros/buffer.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace bitbots_utils {

/**
 * @brief Waits for the transforms to be available
 * @param logger The logger to use for logging
 * @param clock The clock to use for time
 * @param tf_buffer The tf buffer to use
 * @param frames The tf frames to wait for
 * @param root_frame The root frame to transform from
 * @param check_interval Interval in which to check for the frames
 * @param warn_duration Duration after which to warn if the frames are not available
 * @param warn_interval Interval in which to keep warning if the frames are not available
 * @param verbose Can be used to disable the warning messages
 */
void wait_for_tf(const rclcpp::Logger &logger, std::shared_ptr<rclcpp::Clock> clock, tf2_ros::Buffer *tf_buffer,
                 const std::vector<std::string> &frames, const std::string &root_frame,
                 const rclcpp::Duration &check_interval = rclcpp::Duration(0.1s),
                 const rclcpp::Duration &warn_duration = rclcpp::Duration(5.0s),
                 const rclcpp::Duration &warn_interval = rclcpp::Duration(1.0s), bool verbose = true);

/**
 * @brief Get the parameters from other node by calling the get_parameters service
 *
 * @param own_node The node that calls the service
 * @param other_node_name The name of the node that provides the parameters
 * @param parameter_names The names of the parameters to get
 * @param service_timeout The timeout for the service call, if the service call takes longer than this the other node is
 * not available
 * @return std::map<std::string, rclcpp::Parameter> A map of the parameters with the name as key
 */

std::map<std::string, rclcpp::Parameter> get_parameters_from_other_node(rclcpp::Node::SharedPtr own_node,
                                                                        const std::string &other_node_name,
                                                                        const std::vector<std::string> &parameter_names,
                                                                        const std::chrono::seconds &service_timeout);

}  // namespace bitbots_utils

#endif  // BITBOTS_UTILS__UTILS_H_
