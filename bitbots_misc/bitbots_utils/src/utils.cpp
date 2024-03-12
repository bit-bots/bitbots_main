#include "bitbots_utils/utils.hpp"

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

}  // namespace bitbots_utils
