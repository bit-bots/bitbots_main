#ifndef BITBOTS_UTILS__UTILS_H_
#define BITBOTS_UTILS__UTILS_H_

#include <tf2_ros/buffer.h>

#include <bitbots_msgs/msg/parameter_blackboard.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <stdexcept>
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
void wait_for_tf(const rclcpp::Logger& logger, std::shared_ptr<rclcpp::Clock> clock, tf2_ros::Buffer* tf_buffer,
                 const std::vector<std::string>& frames, const std::string& root_frame,
                 const rclcpp::Duration& check_interval = rclcpp::Duration(0.1s),
                 const rclcpp::Duration& warn_duration = rclcpp::Duration(5.0s),
                 const rclcpp::Duration& warn_interval = rclcpp::Duration(1.0s), bool verbose = true);

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
                                                                        const std::string& other_node_name,
                                                                        const std::vector<std::string>& parameter_names,
                                                                        const std::chrono::seconds& service_timeout);

// Topic on which the parameter_blackboard node publishes its complete state
inline const std::string GLOBAL_PARAMETER_BLACKBOARD_TOPIC = "global_parameter_blackboard";

/**
 * @brief QoS for latched topics like the global parameter blackboard or the field maps, so late subscribers get the
 * last message
 */
inline rclcpp::QoS latched_qos() { return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(); }

/**
 * @brief Waits for a message on a latched (transient local) topic and periodically warns while waiting
 *
 * The node is not spun, so this can be used during initialization before the node is added to an executor.
 * Do not call it while an executor is spinning the node, because the executor might take the message first.
 *
 * @param node The node used to create the temporary subscription
 * @param topic The name of the topic, relative names are resolved in the namespace of the node
 * @param warn_interval Interval in which a warning is logged while waiting
 * @return MsgT The received message
 * @throws std::runtime_error If ROS shuts down while waiting
 */
template <typename MsgT>
MsgT wait_for_latched_message(rclcpp::Node::SharedPtr node, const std::string& topic,
                              const std::chrono::seconds& warn_interval = 5s) {
  MsgT msg;
  auto start_time = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    if (rclcpp::wait_for_message(msg, node, topic, warn_interval, latched_qos())) {
      return msg;
    }
    auto waited = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time);
    RCLCPP_WARN(node->get_logger(), "Still waiting for a message on the latched topic '%s' (%lds)", topic.c_str(),
                static_cast<long>(waited.count()));
  }
  throw std::runtime_error("ROS was shut down while waiting for a message on the latched topic '" + topic + "'");
}

/**
 * @brief Get parameters from the global parameter blackboard by waiting for its latched state topic
 *
 * See wait_for_latched_message for the restrictions regarding spinning nodes.
 * It is defined in the header, so only users of it link against the message type support of bitbots_msgs.
 *
 * @param node The node used to create the temporary subscription
 * @param parameter_names The names of the requested parameters (e.g. 'field.size.x')
 * @return std::map<std::string, rclcpp::Parameter> A map of the requested parameters with the name as key
 * @throws std::runtime_error If any of the requested parameters is not set on the blackboard
 */
inline std::map<std::string, rclcpp::Parameter> get_global_parameters(rclcpp::Node::SharedPtr node,
                                                                      const std::vector<std::string>& parameter_names) {
  auto state =
      wait_for_latched_message<bitbots_msgs::msg::ParameterBlackboard>(node, GLOBAL_PARAMETER_BLACKBOARD_TOPIC);

  // Index all parameters of the blackboard by their name
  std::map<std::string, rclcpp::Parameter> available;
  for (const auto& parameter : state.parameters) {
    available[parameter.name] = rclcpp::Parameter::from_parameter_msg(parameter);
  }

  // Collect the requested parameters and report all missing ones at once
  std::map<std::string, rclcpp::Parameter> results;
  std::string missing;
  for (const auto& name : parameter_names) {
    auto it = available.find(name);
    if (it == available.end()) {
      missing += (missing.empty() ? "" : ", ") + name;
    } else {
      results[name] = it->second;
    }
  }
  if (!missing.empty()) {
    throw std::runtime_error("The parameters [" + missing +
                             "] are not set on the global parameter blackboard. Check the configuration loaded by the "
                             "parameter_blackboard node.");
  }
  return results;
}

}  // namespace bitbots_utils

#endif  // BITBOTS_UTILS__UTILS_H_
