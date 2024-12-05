#include <bitbots_utils/utils.hpp>

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
                 const rclcpp::Duration &check_interval, const rclcpp::Duration &warn_duration,
                 const rclcpp::Duration &warn_interval, bool verbose) {
  // Store the beginning time
  auto start_time = clock->now();

  // Endless loop
  while (rclcpp::ok()) {
    try {
      // Check if the frame we want to transform is known yet
      // Apply tf_buffer->_frameExists to all frames and check if all are available otherwise return false (functional)
      // We use _frameExists because it does not spam the console with errors if the frame does not exist...
      if (!std::all_of(frames.begin(), frames.end(),
                       [tf_buffer](std::string frame) { return tf_buffer->_frameExists(frame); })) {
        // Don't do busy waiting
        rclcpp::sleep_for(check_interval.to_chrono<std::chrono::nanoseconds>());
        // Retry
        continue;
      }

      // Check if we can transform from the given root frame to all given frames
      if (!std::all_of(frames.begin(), frames.end(), [tf_buffer, root_frame, check_interval](const std::string frame) {
            return tf_buffer->canTransform(root_frame, frame, rclcpp::Time(0), check_interval);
          })) {
        // Here it is fine not to wait as the canTransform function already includes a timeout
        // Retry
        continue;
      }
      // We can transform to all frames, so we are done
      return;
    } catch (const std::exception &e) {
      if (verbose) {
        RCLCPP_ERROR(logger, "Error while waiting for transforms: %s \n", e.what());
      }
      rclcpp::sleep_for(check_interval.to_chrono<std::chrono::nanoseconds>());
    }

    // Print error message if we waited too long
    auto wait_time = clock->now() - start_time;
    if (verbose && wait_time > warn_duration) {
      RCLCPP_WARN_THROTTLE(logger, *clock, warn_interval.seconds(),
                           "Waiting for transforms took longer than %f seconds", wait_time.seconds());
    }
  }
}

std::map<std::string, rclcpp::Parameter> get_parameters_from_other_node(rclcpp::Node::SharedPtr own_node,
                                                                        const std::string &other_node_name,
                                                                        const std::vector<std::string> &parameter_names,
                                                                        const std::chrono::seconds &service_timeout) {
  // Create a client to the other node
  auto client = own_node->create_client<rcl_interfaces::srv::GetParameters>(other_node_name + "/get_parameters");

  // Wait for the service to be available
  if (!client->wait_for_service(service_timeout)) {
    throw std::runtime_error("Wait for " + other_node_name + " parameter service timed out");
  }

  // Create the request
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = parameter_names;

  // Call the service
  auto future = client->async_send_request(request);
  rclcpp::spin_until_future_complete(own_node, future);
  auto response = future.get();

  // Create a map to store the results
  std::map<std::string, rclcpp::Parameter> results;

  // Fill the map with the received parameters
  for (size_t i = 0; i < parameter_names.size(); i++) {
    // Convert to parameter message fist, so we have less hassle with the params type template
    auto parameter = rcl_interfaces::msg::Parameter();
    parameter.name = parameter_names[i];
    parameter.value = response->values[i];
    results[parameter_names[i]] = rclcpp::Parameter::from_parameter_msg(parameter);
  }

  return results;
}
}  // namespace bitbots_utils
