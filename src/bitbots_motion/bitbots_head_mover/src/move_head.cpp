#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <bitbots_head_mover/head_parameters.hpp>
#include <bitbots_msgs/action/look_at.hpp>
#include <bitbots_msgs/msg/head_mode.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <bitbots_splines/smooth_spline.hpp>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace move_head {

#define DEG_TO_RAD M_PI / 180
#define YAW_PITCH_JOINT_Z_DISTANCE 0.0275

using LookAtGoal = bitbots_msgs::action::LookAt;
using LookAtGoalHandle = rclcpp_action::ServerGoalHandle<LookAtGoal>;

class HeadMover {
  std::shared_ptr<rclcpp::Node> node_;

  // Declare subscriber
  rclcpp::Subscription<bitbots_msgs::msg::HeadMode>::SharedPtr head_mode_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ball_filter_subscriber_;

  // Declare publisher
  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr position_publisher_;

  // Declare tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Declare variables
  uint head_mode_ = bitbots_msgs::msg::HeadMode::LOOK_FORWARD;
  std::optional<sensor_msgs::msg::JointState> current_joint_state_;
  geometry_msgs::msg::PoseWithCovarianceStamped tf_precision_pose_;

  // Declare parameters and parameter listener
  move_head::Params params_;
  std::shared_ptr<move_head::ParamListener> param_listener_;

  // Declare timer that executes the main loop
  rclcpp::TimerBase::SharedPtr timer_;

  // Declare variable for the current search pattern
  std::vector<std::pair<double, double>> pattern_;
  // Store previous head mode
  uint prev_head_mode_ = -1;

  // Duration of one full search pattern cycle (seconds)
  double cycle_time_ = 0.0;

  // Spline trajectory for search patterns
  bitbots_splines::SmoothSpline yaw_spline_;
  bitbots_splines::SmoothSpline pitch_spline_;
  double spline_duration_ = 0.0;
  // Duration of the transition segment from the current head position into the pattern (prepended to the cycle)
  double transition_duration_ = 0.0;
  rclcpp::Time spline_start_time_;
  bool spline_valid_ = false;

  // World model state
  geometry_msgs::msg::PoseWithCovarianceStamped ball_position_;

  // Action server for the look at action
  rclcpp_action::Server<LookAtGoal>::SharedPtr action_server_;
  bool action_running_ = false;

 public:
  HeadMover() : node_(std::make_shared<rclcpp::Node>("head_mover")) {
    // Initialize publisher for head motor goals
    position_publisher_ = node_->create_publisher<bitbots_msgs::msg::JointCommand>("head_motor_goals", 10);

    // Initialize subscriber for head mode
    head_mode_subscriber_ = node_->create_subscription<bitbots_msgs::msg::HeadMode>(
        "head_mode", 10, [this](const bitbots_msgs::msg::HeadMode::SharedPtr msg) {
          // Cppcheck misses the lambda and thinks we do this in the constructor itself
          // cppcheck-suppress useInitializationList
          head_mode_ = msg->head_mode;
        });

    // Initialize subscriber for the current joint states of the robot
    joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          // cppcheck-suppress useInitializationList
          current_joint_state_ = *msg;
        });

    // Initialize subscriber for the ball filter
    ball_filter_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "ball_position_relative_filtered", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          // cppcheck-suppress useInitializationList
          ball_position_ = *msg;
        });

    // Initialize with a valid frame
    ball_position_.header.frame_id = "base_footprint";

    // Create parameter listener and load initial set of parameters
    param_listener_ = std::make_shared<move_head::ParamListener>(node_);
    params_ = param_listener_->get_params();

    // Create tf buffer and listener to update it
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize action server for look at action
    action_server_ = rclcpp_action::create_server<LookAtGoal>(
        node_, "look_at_goal", std::bind(&HeadMover::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&HeadMover::handle_cancel, this, std::placeholders::_1),
        std::bind(&HeadMover::handle_accepted, this, std::placeholders::_1));

    // Initialize timer for main loop
    timer_ = rclcpp::create_timer(node_, node_->get_clock(), 50ms, [this] { behave(); });
  }

  /***
   * @brief Handles the goal request for the look at action
   *
   * @param uuid
   * @param goal
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const LookAtGoal::Goal> goal) {
    // Avoid unused parameter warning
    (void)uuid;
    RCLCPP_DEBUG(node_->get_logger(), "Received goal request");

    // Bring the goal point into the planning frame
    geometry_msgs::msg::PointStamped head_yaw_point;
    try {
      head_yaw_point = tf_buffer_->transform(goal->look_at_position, "head_yaw_link", tf2::durationFromSec(0.9));
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "Could not transform goal point: %s", ex.what());
      return rclcpp_action::GoalResponse::REJECT;
    }

    // RCLCPP_DEBUG(node_->get_logger(), "yaw point, pitch point" << head_yaw_point.point << " " <<
    // head_pitch_point.point);

    // Get the motor goals that are needed to look at the point
    std::pair<double, double> yaw_pitch = get_motor_goals_from_point(head_yaw_point.point);

    // Check whether the goal is in range yaw and pitch wise
    bool goal_not_in_range = check_head_collision(yaw_pitch.first, yaw_pitch.second);

    // Check whether the action goal is valid and can be executed
    // cppcheck-suppress knownConditionTrueFalse
    if (action_running_ || goal_not_in_range ||
        !(params_.max_yaw[0] < yaw_pitch.first && yaw_pitch.first < params_.max_yaw[1]) ||
        !(params_.max_pitch[0] < yaw_pitch.second && yaw_pitch.second < params_.max_pitch[1])) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Handles the cancel request for the look at action
   *
   * @param goal_handle
   * @return rclcpp_action::CancelResponse
   */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<LookAtGoalHandle> goal_handle) {
    // Avoid unused parameter warning
    (void)goal_handle;
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
    // Set the action_running_ flag to false, so that the action can be executed again
    action_running_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Handles the accepted request for the look at action
   *
   * @param goal_handle
   */
  void handle_accepted(const std::shared_ptr<LookAtGoalHandle> goal_handle) {
    // Spawn a new thread that executes the look at action until we reach the goal
    std::thread{std::bind(&HeadMover::execute_look_at, this, std::placeholders::_1), goal_handle}.detach();
  }

  /**
   * @brief Executes the look at action that looks at a specific point in a given frame until the goal is reached or the
   * action is canceled
   *
   * @param goal_handle
   */
  void execute_look_at(const std::shared_ptr<LookAtGoalHandle> goal_handle) {
    // Yeah seems like we are executing the action
    action_running_ = true;

    RCLCPP_INFO(node_->get_logger(), "Executing goal");

    // Get the goal from the goal handle
    const auto goal = goal_handle->get_goal();

    // Create feedback and result messages
    auto feedback = std::make_shared<LookAtGoal::Feedback>();
    // Flag that indicates whether the action was successful yet
    bool success = false;
    auto result = std::make_shared<LookAtGoal::Result>();

    // Execute the action until we reach the goal or the action is canceled
    while (!success && rclcpp::ok()) {
      RCLCPP_DEBUG(node_->get_logger(), "Looking at point");

      // Check if the action was canceled and if so, set the result accordingly
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(node_->get_logger(), "Goal was canceled");
        return;
      }

      // Look at the goal point
      success = look_at(goal->look_at_position);

      // Publish feedback to the client
      goal_handle->publish_feedback(feedback);  // TODO: currently feedback is empty
    }

    // If we reach this point, the action was successful
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
    }

    // Set the action_running_ flag to false, so that the action can be executed again
    action_running_ = false;
  }

  /**
   * @brief Slows down the speed of the joint that needs to travel less distance so both joints reach the goal at the
   * same time
   *
   * @param delta_faster_joint The delta of the joint that needs to travel less distance and therefore reaches the goal
   * faster
   * @param delta_joint The delta of the joint that needs to travel more distance and therefore reaches the goal slower
   * @param speed The maximum speed of the faster joint (the joint that needs to travel less distance)
   * @return double The adjusted speed of the faster joint
   */
  double calculate_lower_speed(double delta_faster_joint, double delta_joint, double speed) {
    double estimated_time = delta_faster_joint / speed;
    if (estimated_time != 0) {
      return delta_joint / estimated_time;
    } else {
      return 0;
    }
  }

  /**
   * @brief Send the goal positions to the head motors, but resolve collisions with the body if necessary.
   *
   */
  bool send_motor_goals(double yaw_position, double pitch_position, bool resolve_collision, double yaw_speed = 1.5,
                        double pitch_speed = 1.5, double current_yaw_position = 0.0,
                        double current_pitch_position = 0.0, bool clip = true) {
    // Debug log the target yaw and pitch position
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "target yaw/pitch: " << yaw_position << "/" << pitch_position);

    // Clip the target yaw and pitch position at the maximum yaw and pitch values as defined in the parameters
    if (clip) {
      std::tie(yaw_position, pitch_position) = pre_clip(yaw_position, pitch_position);
    }

    // Resolve collisions if necessary
    if (resolve_collision) {
      // Call behavior that resolves collisions and might change the target yaw and pitch position
      bool success = avoid_collision_on_path(yaw_position, pitch_position, current_yaw_position, current_pitch_position,
                                             yaw_speed, pitch_speed);
      // Report error message of we were not able to move to an alternative collision free position
      if (!success) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                     "Unable to resolve head collision");
      }
      return success;
    } else {
      // Move the head to the target position but adjust the speed of the joints so both reach the goal at the same time
      move_head_to_position_with_speed_adjustment(yaw_position, pitch_position, current_yaw_position,
                                                  current_pitch_position, yaw_speed, pitch_speed);
      return true;
    }
  }

  /**
   * @brief Applies clipping to the yaw and pitch values based on the loaded config parameters
   *
   */
  std::pair<double, double> pre_clip(double yaw, double pitch) {
    double new_yaw = std::clamp(yaw, params_.max_yaw[0], params_.max_yaw[1]);
    double new_pitch = std::clamp(pitch, params_.max_pitch[0], params_.max_pitch[1]);
    return {new_yaw, new_pitch};
  }

  /**
   * @brief Tries to move the head to the target position but resolves collisions with the body if necessary.
   *
   */
  bool avoid_collision_on_path(double goal_yaw, double goal_pitch, double current_yaw, double current_pitch,
                               double yaw_speed, double pitch_speed, int max_depth = 4, int depth = 0) {
    // Check if we reached the maximum depth of the recursion and if so, return false
    if (depth > max_depth) {
      return false;
    }

    // Calculate the distance between the current and the goal position
    double distance = sqrt(pow(goal_yaw - current_yaw, 2) + pow(goal_pitch - current_pitch, 2));

    // Calculate the number of steps we need to take to reach the goal position
    // This assumes that we move 3 degrees per step
    int step_count = distance / (3 * DEG_TO_RAD);

    // Calculate path by performing linear interpolation between the current and the goal position
    std::vector<std::pair<double, double>> yaw_and_pitch_steps;
    for (int i = 0; i < step_count; i++) {
      yaw_and_pitch_steps.push_back({current_yaw + (goal_yaw - current_yaw) / step_count * i,
                                     current_pitch + (goal_pitch - current_pitch) / step_count * i});
    }

    // Check if we have collisions on our path
    for (int i = 0; i < step_count; i++) {
      // cppcheck-suppress knownConditionTrueFalse
      if (check_head_collision(yaw_and_pitch_steps[i].first, yaw_and_pitch_steps[i].second)) {
        // If we have a collision, try to move the head to an alternative position
        // The new position looks 10 degrees further up and is less likely to have a collision with the body
        // Also increase the depth of the recursion as this is a new attempt to move the head to the goal position
        return avoid_collision_on_path(goal_yaw, goal_pitch + 10 * DEG_TO_RAD, current_yaw, current_pitch, yaw_speed,
                                       pitch_speed, max_depth, depth + 1);
      }
    }

    // We do not have any collisions on our path, so we can move the head to the goal position
    move_head_to_position_with_speed_adjustment(goal_yaw, goal_pitch, current_yaw, current_pitch, yaw_speed,
                                                pitch_speed);
    return true;
  }

  /**
   * @brief Checks if the head collides with the body at a given yaw and pitch position
   */
  bool check_head_collision(double yaw, double pitch) {
    // TODO we do not have a collision model for the pi plus head yet, so we need to implement this function properly
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Collision checking for the pi plus head is not implemented yet");
    return false;
  }

  /**
   * @brief Move the head to the target position but adjust the speed of the joints so both reach the goal at the same
   * time
   */
  void move_head_to_position_with_speed_adjustment(double goal_yaw, double goal_pitch, double current_yaw,
                                                   double current_pitch, double yaw_speed, double pitch_speed) {
    // Calculate the delta between the current and the goal positions
    double delta_yaw = std::abs(goal_yaw - current_yaw);
    double delta_pitch = std::abs(goal_pitch - current_pitch);
    // Check which axis has to move further and adjust the speed of the other axis so both reach the goal at the same
    // time
    if (delta_yaw > delta_pitch) {
      // Slow down the pitch axis to match the time it takes for the yaw axis to reach the goal
      pitch_speed = std::min(pitch_speed, calculate_lower_speed(delta_yaw, delta_pitch, yaw_speed));
    } else {
      // Slow down the yaw axis to match the time it takes for the pitch axis to reach the goal
      yaw_speed = std::min(yaw_speed, calculate_lower_speed(delta_pitch, delta_yaw, pitch_speed));
    }

    // Send the motor goals including the position, speed and acceleration
    bitbots_msgs::msg::JointCommand pos_msg;
    pos_msg.header.stamp = rclcpp::Clock().now();
    pos_msg.joint_names = {"head_yaw_joint", "head_pitch_joint"};
    pos_msg.positions = {goal_yaw, goal_pitch};
    pos_msg.velocities = {yaw_speed, pitch_speed};
    pos_msg.accelerations = {params_.max_acceleration_yaw, params_.max_acceleration_pitch};
    pos_msg.max_torques = {10, 10};

    position_publisher_->publish(pos_msg);
  }

  /**
   * @brief Returns the current position of the head motors
   */
  std::pair<double, double> get_head_position() {
    double head_yaw = 0.0;
    double head_pitch = 0.0;

    // Iterate over all joints and find the head yaw and pitch joints
    for (size_t i = 0; i < current_joint_state_->name.size(); i++) {
      if (current_joint_state_->name[i] == "head_yaw_joint") {
        head_yaw = current_joint_state_->position[i];
      } else if (current_joint_state_->name[i] == "head_pitch_joint") {
        head_pitch = current_joint_state_->position[i];
      }
    }
    return {head_yaw, head_pitch};
  }

  /**
   * @brief Converts a scanline number to a pitch angle
   */
  double lineAngle(int line, int line_count, double min_angle, double max_angle) {
    // Get the angular delta that is covered by the scanlines in the pitch axis
    double delta = std::abs(max_angle - min_angle);
    // Calculate the angular step size between two scanlines
    double steps = delta / (line_count - 1);
    // Calculate the pitch angle of the given scanline
    return steps * line + min_angle;
  }

  /**
   * @brief Performs a linear interpolation between the min and max yaw values and returns the interpolated steps
   */
  std::vector<std::pair<double, double>> interpolatedSteps(int steps, double pitch, double min_yaw, double max_yaw) {
    // Handle edge case where we do not need to interpolate
    if (steps == 0) {
      return {};
    }
    // Add one to the step count as we need to include the min and max yaw values
    steps += 1;
    // Create a vector that stores the interpolated steps
    std::vector<std::pair<double, double>> output_points;
    // Calculate the delta between the min and max yaw values
    double delta = std::abs(max_yaw - min_yaw);
    // Calculate the step size between two interpolated steps
    double step_size = delta / steps;
    // Iterate over all steps and calculate the interpolated yaw values
    for (int i = 1; i <= steps; i++) {
      double yaw = min_yaw + step_size * i;
      output_points.emplace_back(yaw, pitch);
    }
    return output_points;
  }

  /**
   * @brief Generates a parameterized search pattern
   */
  std::vector<std::pair<double, double>> generatePattern(int line_count, double max_horizontal_angle_left,
                                                         double max_horizontal_angle_right,
                                                         double max_vertical_angle_up, double max_vertical_angle_down,
                                                         double reduce_last_scanline = 1.0,
                                                         int interpolation_steps = 0) {
    // Store the keyframes of the search pattern
    std::vector<std::pair<double, double>> keyframes;
    // Store the state of the generation process
    bool down_direction = true;   // true = decreasing line (toward top), false = increasing line (toward bottom)
    bool right_side = false;      // true = right, false = left
    bool right_direction = true;  // true = moving right, false = moving left; alternates per scan line
    int line = line_count - 1;
    // Calculate the number of iterations that are needed to generate the search pattern
    int iterations = std::max(line_count * 4 - 4, 2);
    // Iterate over all iterations and generate the search pattern
    for (int i = 0; i < iterations; i++) {
      // Get the maximum yaw values (left and right) for the current yaw position
      // Select the relevant one based on the current side we are on
      double current_yaw;
      if (right_side) {
        current_yaw = max_horizontal_angle_right;
      } else {
        current_yaw = max_horizontal_angle_left;
      }

      // Get the current pitch angle based on the current line we are on
      double current_pitch = lineAngle(line, line_count, max_vertical_angle_up, max_vertical_angle_down);

      // Store the keyframe
      keyframes.push_back({current_yaw, current_pitch});

      // Check if we move horizontally or vertically in the pattern
      if (right_side != right_direction) {
        // We move horizontally, so we might need to interpolate between the current and the next keyframe
        std::vector<std::pair<double, double>> interpolated_points = interpolatedSteps(
            interpolation_steps, current_pitch, max_horizontal_angle_right, max_horizontal_angle_left);
        // Reverse the order of the interpolated points if we are moving to the right
        if (right_direction) {
          std::reverse(interpolated_points.begin(), interpolated_points.end());
        }
        // Add the interpolated points to the keyframes
        keyframes.insert(keyframes.end(), interpolated_points.begin(), interpolated_points.end());
        // Change the direction we are moving in
        right_side = right_direction;

      } else {
        // Flip the scan direction so the next line scans the opposite way (boustrophedon)
        right_direction = !right_direction;
        // Advance to the next scan line
        if (down_direction) {
          line -= 1;
        } else {
          line += 1;
        }
        // Flip vertical direction when we reach either edge
        if (line <= 0 || line >= line_count - 1) {
          down_direction = !down_direction;
        }
      }
    }

    // Reduce the last scanline by a given factor
    for (auto& keyframe : keyframes) {
      if (std::abs(keyframe.second - max_vertical_angle_down) < 1e-6) {
        keyframe = {keyframe.first * reduce_last_scanline, max_vertical_angle_down};
      }
    }
    return keyframes;
  }

  /**
   * @brief Calculates the motor goals that are needed to look at a given point using the inverse kinematics
   */
  std::pair<double, double> get_motor_goals_from_point(geometry_msgs::msg::Point head_yaw_point) {
    double x = head_yaw_point.x;
    double y = head_yaw_point.y;
    double z = head_yaw_point.z;

    double head_yaw = atan2(y, x);

    double head_pitch = -atan2(z - YAW_PITCH_JOINT_Z_DISTANCE, sqrt(x * x + y * y));

    return {head_yaw, head_pitch};
  }

  /**
   * @brief Looks at a given point and returns true if the goal position was reached
   */
  bool look_at(geometry_msgs::msg::PointStamped point, double min_yaw_delta = 0.02, double min_pitch_delta = 0.02) {
    try {
      // Transform the point into the planning frame
      geometry_msgs::msg::PointStamped head_yaw_point =
          tf_buffer_->transform(point, "head_yaw_link", tf2::durationFromSec(0.9));

      // Get the motor goals that are needed to look at the point from the inverse kinematics
      std::pair<double, double> yaw_pitch = get_motor_goals_from_point(head_yaw_point.point);
      // Get the current head position
      std::pair<double, double> current_yaw_pitch = get_head_position();

      // Check if we reached the goal position
      if (std::abs(yaw_pitch.first - current_yaw_pitch.first) > min_yaw_delta ||
          std::abs(yaw_pitch.second - current_yaw_pitch.second) > min_pitch_delta) {
        // Send the motor goals to the head motors
        send_motor_goals(yaw_pitch.first, yaw_pitch.second, true, params_.look_at.yaw_speed,
                         params_.look_at.pitch_speed);
        // Return false as we did not reach the goal position yet
        return false;
      }
      // Return true as we reached the goal position
      return true;
    } catch (tf2::TransformException& ex) {
      // Report error message if we were not able to transform the point
      RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", ex.what());
      // We obviously did not reach the goal position
      return false;
    }
  }

  /**
   * @brief Builds an open-loop SmoothSpline trajectory from the current search pattern.
   *
   * The trajectory starts at the current head position and smoothly transitions into the
   * pattern, so switching head modes does not result in an abrupt movement. Waypoint
   * timestamps are distributed proportionally to Euclidean arc length so the cycle
   * (excluding the transition) takes exactly cycle_time_ seconds. The loop is closed by
   * appending the first waypoint at the end.
   */
  void build_spline_trajectory() {
    yaw_spline_ = bitbots_splines::SmoothSpline();
    pitch_spline_ = bitbots_splines::SmoothSpline();
    spline_valid_ = false;
    transition_duration_ = 0.0;

    if (pattern_.empty() || cycle_time_ <= 0.0) {
      return;
    }

    // Convert all keypoints to radians and close the loop
    std::vector<std::pair<double, double>> pts_rad;
    pts_rad.reserve(pattern_.size() + 1);
    for (const auto& kf : pattern_) {
      pts_rad.emplace_back(kf.first * DEG_TO_RAD, kf.second * DEG_TO_RAD);
    }
    pts_rad.push_back(pts_rad[0]);

    // Compute segment arc lengths and total
    std::vector<double> lengths;
    lengths.reserve(pts_rad.size() - 1);
    double total_length = 0.0;
    for (size_t i = 1; i < pts_rad.size(); i++) {
      double dyaw = pts_rad[i].first - pts_rad[i - 1].first;
      double dpitch = pts_rad[i].second - pts_rad[i - 1].second;
      double len = std::sqrt(dyaw * dyaw + dpitch * dpitch);
      lengths.push_back(len);
      total_length += len;
    }

    double t = 0.0;

    // Prepend a transition segment from the current head position to the first waypoint
    // of the pattern, so we don't jump there abruptly when the head mode changes.
    // Its duration is based on the distance and the configured transition speed.
    auto [current_yaw, current_pitch] = get_head_position();
    double transition_distance =
        std::sqrt(std::pow(pts_rad[0].first - current_yaw, 2) + std::pow(pts_rad[0].second - current_pitch, 2));
    transition_duration_ = transition_distance / params_.transition_speed;
    if (transition_duration_ > 0.0) {
      yaw_spline_.addPoint(t, current_yaw);
      pitch_spline_.addPoint(t, current_pitch);
      t = transition_duration_;
    }

    // Add waypoints with timestamps proportional to arc length
    yaw_spline_.addPoint(t, pts_rad[0].first);
    pitch_spline_.addPoint(t, pts_rad[0].second);
    for (size_t i = 0; i < lengths.size(); i++) {
      double fraction = (total_length > 0.0) ? lengths[i] / total_length : 1.0 / static_cast<double>(lengths.size());
      t += fraction * cycle_time_;
      yaw_spline_.addPoint(t, pts_rad[i + 1].first);
      pitch_spline_.addPoint(t, pts_rad[i + 1].second);
    }

    spline_duration_ = cycle_time_;
    yaw_spline_.computeSplines();
    pitch_spline_.computeSplines();
    spline_start_time_ = node_->now();
    spline_valid_ = true;
  }

  /**
   * @brief Evaluates the pre-built spline trajectory at the current time and publishes
   * the resulting joint position and velocity as open-loop motor goals.
   */
  void perform_search_pattern() {
    if (!spline_valid_ || spline_duration_ <= 0.0) {
      return;
    }

    // Play the transition from the previous head position once, then loop only over the cyclic part of the trajectory
    double elapsed = (node_->now() - spline_start_time_).seconds();
    double t;
    if (elapsed <= transition_duration_) {
      t = elapsed;
    } else {
      t = transition_duration_ + fmod(elapsed - transition_duration_, spline_duration_);
    }

    double goal_yaw = yaw_spline_.pos(t);
    double goal_pitch = pitch_spline_.pos(t);
    std::tie(goal_yaw, goal_pitch) = pre_clip(goal_yaw, goal_pitch);

    double yaw_vel = std::abs(yaw_spline_.vel(t));
    double pitch_vel = std::abs(pitch_spline_.vel(t));

    bitbots_msgs::msg::JointCommand pos_msg;
    pos_msg.header.stamp = node_->get_clock()->now();
    pos_msg.joint_names = {"head_yaw_joint", "head_pitch_joint"};
    pos_msg.positions = {goal_yaw, goal_pitch};
    pos_msg.velocities = {yaw_vel, pitch_vel};
    pos_msg.accelerations = {params_.max_acceleration_yaw, params_.max_acceleration_pitch};
    pos_msg.max_torques = {10, 10};

    position_publisher_->publish(pos_msg);
  }

  /**
   * @brief Callback for the ticks of the main loop
   */
  void behave() {
    // Get the current head mode
    uint curr_head_mode = head_mode_;

    // Pull the parameters from the parameter server
    params_ = param_listener_->get_params();

    // Check if we received the joint states yet and if not, return
    if (!current_joint_state_) {
      return;
    }

    // Check if the head mode changed and if so, update the search pattern
    if (prev_head_mode_ != curr_head_mode) {
      switch (curr_head_mode) {
        case bitbots_msgs::msg::HeadMode::TRACK_BALL:
        case bitbots_msgs::msg::HeadMode::DONT_MOVE:
          // Nothing to do if we go into track ball or dont move mode
          break;
        case bitbots_msgs::msg::HeadMode::SEARCH_BALL_PENALTY:
          cycle_time_ = params_.search_patterns.search_ball_penalty.cycle_time;
          pattern_ = generatePattern(params_.search_patterns.search_ball_penalty.scan_lines,
                                     params_.search_patterns.search_ball_penalty.yaw_max[0],
                                     params_.search_patterns.search_ball_penalty.yaw_max[1],
                                     params_.search_patterns.search_ball_penalty.pitch_max[0],
                                     params_.search_patterns.search_ball_penalty.pitch_max[1],
                                     params_.search_patterns.search_ball_penalty.reduce_last_scanline);
          break;

        case bitbots_msgs::msg::HeadMode::SEARCH_FIELD_FEATURES:
          cycle_time_ = params_.search_patterns.search_field_features.cycle_time;
          pattern_ = generatePattern(params_.search_patterns.search_field_features.scan_lines,
                                     params_.search_patterns.search_field_features.yaw_max[0],
                                     params_.search_patterns.search_field_features.yaw_max[1],
                                     params_.search_patterns.search_field_features.pitch_max[0],
                                     params_.search_patterns.search_field_features.pitch_max[1],
                                     params_.search_patterns.search_field_features.reduce_last_scanline);
          break;

        case bitbots_msgs::msg::HeadMode::SEARCH_FRONT:
          cycle_time_ = params_.search_patterns.search_front.cycle_time;
          pattern_ = generatePattern(
              params_.search_patterns.search_front.scan_lines, params_.search_patterns.search_front.yaw_max[0],
              params_.search_patterns.search_front.yaw_max[1], params_.search_patterns.search_front.pitch_max[0],
              params_.search_patterns.search_front.pitch_max[1],
              params_.search_patterns.search_front.reduce_last_scanline);
          break;

        case bitbots_msgs::msg::HeadMode::LOOK_FORWARD:
          cycle_time_ = params_.search_patterns.look_forward.cycle_time;
          pattern_ = generatePattern(
              params_.search_patterns.look_forward.scan_lines, params_.search_patterns.look_forward.yaw_max[0],
              params_.search_patterns.look_forward.yaw_max[1], params_.search_patterns.look_forward.pitch_max[0],
              params_.search_patterns.look_forward.pitch_max[1],
              params_.search_patterns.look_forward.reduce_last_scanline);
          break;

        default:
          return;
      }

      // Store the current head mode as the previous head mode to detect changes
      prev_head_mode_ = curr_head_mode;

      // Build the spline trajectory from the new pattern. It starts with a smooth transition from
      // the current head position, so we also rebuild it when we re-enter a search pattern from
      // e.g. ball tracking instead of continuing the old trajectory with an abrupt movement.
      if (curr_head_mode != bitbots_msgs::msg::HeadMode::TRACK_BALL &&
          curr_head_mode != bitbots_msgs::msg::HeadMode::DONT_MOVE) {
        build_spline_trajectory();
      }
    }
    // Check if no look at action is running or if the head mode is DONT_MOVE
    // if this is not the case, perform the search pattern
    if (!action_running_ && curr_head_mode != bitbots_msgs::msg::HeadMode::DONT_MOVE)  // here DONT_MOVE
                                                                                       // is implemented
    {
      // If we are in search track mode look at the ball
      if (curr_head_mode == bitbots_msgs::msg::HeadMode::TRACK_BALL) {
        // Convert the ball position to a PointStamped
        geometry_msgs::msg::PointStamped look_at_point;
        look_at_point.header = ball_position_.header;
        look_at_point.point = ball_position_.pose.pose.position;
        // Try to look at the ball
        look_at(look_at_point);
      } else {
        // Execute the search pattern
        perform_search_pattern();
      }
    }
  }

  /**
   * @brief A getter that returns the node
   */
  std::shared_ptr<rclcpp::Node> get_node() { return node_; }
};
}  // namespace move_head

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::experimental::executors::EventsExecutor exec;
  auto head_mover = std::make_shared<move_head::HeadMover>();
  exec.add_node(head_mover->get_node());
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
