#include <bio_ik/bio_ik.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <bio_ik_msgs/msg/ik_response.hpp>
#include <bitbots_msgs/action/look_at.hpp>
#include <bitbots_msgs/msg/head_mode.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "head_parameters.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace move_head {

#define DEG_TO_RAD M_PI / 180

using LookAtGoal = bitbots_msgs::action::LookAt;
using LookAtGoalHandle = rclcpp_action::ServerGoalHandle<LookAtGoal>;

class HeadMover {
  std::shared_ptr<rclcpp::Node> node_;

  // Declare subscriber
  rclcpp::Subscription<bitbots_msgs::msg::HeadMode>::SharedPtr head_mode_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  // Declare publisher
  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr position_publisher_;

  // Declare tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Declare variables
  uint head_mode_ = bitbots_msgs::msg::HeadMode::LOOK_FORWARD;
  std::optional<sensor_msgs::msg::JointState> current_joint_state_;
  geometry_msgs::msg::PoseWithCovarianceStamped tf_precision_pose_;

  // Declare robot model and planning scene for moveit
  robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  moveit::core::RobotStatePtr collision_state_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningScenePtr planning_scene_;

  // Declare parameters and parameter listener
  move_head::Params params_;
  std::shared_ptr<move_head::ParamListener> param_listener_;

  // Declare timer that executes the main loop
  rclcpp::TimerBase::SharedPtr timer_;

  // Declare variable for the current search pattern
  std::vector<std::pair<double, double>> pattern_;
  // Store previous head mode
  uint prev_head_mode_ = -1;

  // Declare variables for the current search patterns parameters
  double threshold_ = 0;
  int index_ = 0;
  double pan_speed_ = 0;
  double tilt_speed_ = 0;

  // Action server for the look at action
  rclcpp_action::Server<LookAtGoal>::SharedPtr action_server_;
  bool action_running_ = false;

 public:
  HeadMover() : node_(std::make_shared<rclcpp::Node>("head_mover")) {
    // Initialize publisher for head motor goals
    position_publisher_ = node_->create_publisher<bitbots_msgs::msg::JointCommand>("head_motor_goals", 10);

    // Initialize subscriber for head mode
    head_mode_subscriber_ = node_->create_subscription<bitbots_msgs::msg::HeadMode>(
        "head_mode", 10, [this](const bitbots_msgs::msg::HeadMode::SharedPtr msg) { head_mode_callback(msg); });

    // Initialize subscriber for the current joint states of the robot
    joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, [this](const sensor_msgs::msg::JointState::SharedPtr msg) { joint_state_callback(msg); });

    // Create parameter listener and load initial set of parameters
    param_listener_ = std::make_shared<move_head::ParamListener>(node_);
    params_ = param_listener_->get_params();

    // Create a seperate node for moveit, this way we can use rqt to change parameters,
    // because some moveit parameters break the gui
    auto moveit_node = std::make_shared<rclcpp::Node>("moveit_head_mover_node");

    // Get the parameters from the move_group node
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_, "/move_group");
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    }

    // Extract the robot_description
    rcl_interfaces::msg::ListParametersResult parameter_list =
        parameters_client->list_parameters({"robot_description_kinematics"}, 10);

    // Set the robot description parameters in the moveit node
    auto copied_parameters = parameters_client->get_parameters(parameter_list.names);
    for (auto& parameter : copied_parameters) {
      moveit_node->declare_parameter(parameter.get_name(), parameter.get_type());
      moveit_node->set_parameter(parameter);
    }

    // Load robot description / robot model into moveit
    std::string robot_description = "robot_description";
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
        robot_model_loader::RobotModelLoader(moveit_node, robot_description, true));
    robot_model_ = loader_->getModel();
    if (!robot_model_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "failed to load robot model %s. Did you start the "
                   "blackboard (bitbots_bringup base.launch)?",
                   robot_description.c_str());
    }

    // Recreate robot state
    robot_state_.reset(new moveit::core::RobotState(robot_model_));
    robot_state_->setToDefaultValues();

    // Recreate collision state
    collision_state_.reset(new moveit::core::RobotState(robot_model_));
    collision_state_->setToDefaultValues();

    // Get planning scene for collision checking
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(moveit_node, loader_);
    planning_scene_ = planning_scene_monitor_->getPlanningScene();
    if (!planning_scene_) {
      RCLCPP_ERROR_ONCE(node_->get_logger(), "failed to connect to planning scene");
    }

    // Create tf buffer and listener to update it
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize variables
    threshold_ = params_.position_reached_threshold * DEG_TO_RAD;

    // Initialize action server for look at action
    action_server_ = rclcpp_action::create_server<LookAtGoal>(
        node_, "look_at_goal", std::bind(&HeadMover::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&HeadMover::handle_cancel, this, std::placeholders::_1),
        std::bind(&HeadMover::handle_accepted, this, std::placeholders::_1));

    // Initialize timer for main loop
    timer_ = rclcpp::create_timer(node_, node_->get_clock(), 50ms, [this] { behave(); });
  }

  /**
   * @brief Callback used to update the head mode
   */
  void head_mode_callback(const bitbots_msgs::msg::HeadMode::SharedPtr msg) { head_mode_ = msg->head_mode; }

  /**
   * @brief Callback used to get updates of the current joint states of the robot
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) { current_joint_state_ = *msg; }

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
    geometry_msgs::msg::PointStamped new_point;
    try {
      new_point =
          tf_buffer_->transform(goal->look_at_position, planning_scene_->getPlanningFrame(), tf2::durationFromSec(0.9));
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "Could not transform goal point: %s", ex.what());
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Get the motor goals that are needed to look at the point
    std::pair<double, double> pan_tilt = get_motor_goals_from_point(new_point.point);

    // Check whether the goal is in range pan and tilt wise
    bool goal_not_in_range = check_head_collision(pan_tilt.first, pan_tilt.second);

    // Check whether the action goal is valid and can be executed
    if (action_running_ || goal_not_in_range ||
        !(params_.max_pan[0] < pan_tilt.first && pan_tilt.first < params_.max_pan[1]) ||
        !(params_.max_tilt[0] < pan_tilt.second && pan_tilt.second < params_.max_tilt[1])) {
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
      RCLCPP_INFO(node_->get_logger(), "Looking at point");

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
  bool send_motor_goals(double pan_position, double tilt_position, bool resolve_collision, double pan_speed = 1.5,
                        double tilt_speed = 1.5, double current_pan_position = 0.0, double current_tilt_position = 0.0,
                        bool clip = true) {
    // Debug log the target pan and tilt position
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "target pan/tilt: " << pan_position << "/" << tilt_position);

    // Clip the target pan and tilt position at the maximum pan and tilt values as defined in the parameters
    if (clip) {
      std::tie(pan_position, tilt_position) = pre_clip(pan_position, tilt_position);
    }

    // Resolve collisions if necessary
    if (resolve_collision) {
      // Call behavior that resolves collisions and might change the target pan and tilt position
      bool success = avoid_collision_on_path(pan_position, tilt_position, current_pan_position, current_tilt_position,
                                             pan_speed, tilt_speed);
      // Report error message of we were not able to move to an alternative collision free position
      if (!success) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                     "Unable to resolve head collision");
      }
      return success;
    } else {
      // Move the head to the target position but adjust the speed of the joints so both reach the goal at the same time
      move_head_to_position_with_speed_adjustment(pan_position, tilt_position, current_pan_position,
                                                  current_tilt_position, pan_speed, tilt_speed);
      return true;
    }
  }

  /**
   * @brief Applies clipping to the pan and tilt values based on the loaded config parameters
   *
   */
  std::pair<double, double> pre_clip(double pan, double tilt) {
    double new_pan = std::clamp(pan, params_.max_pan[0], params_.max_pan[1]);
    double new_tilt = std::clamp(tilt, params_.max_tilt[0], params_.max_tilt[1]);
    return {new_pan, new_tilt};
  }

  /**
   * @brief Tries to move the head to the target position but resolves collisions with the body if necessary.
   *
   */
  bool avoid_collision_on_path(double goal_pan, double goal_tilt, double current_pan, double current_tilt,
                               double pan_speed, double tilt_speed, int max_depth = 4, int depth = 0) {
    // Check if we reached the maximum depth of the recursion and if so, return false
    if (depth > max_depth) {
      return false;
    }

    // Calculate the distance between the current and the goal position
    double distance = sqrt(pow(goal_pan - current_pan, 2) - pow(goal_tilt - current_tilt, 2));

    // Calculate the number of steps we need to take to reach the goal position
    // This assumes that we move 3 degrees per step
    int step_count = distance / (3 * DEG_TO_RAD);

    // Calculate path by performing linear interpolation between the current and the goal position
    std::vector<std::pair<double, double>> pan_and_tilt_steps;
    for (int i = 0; i < step_count; i++) {
      pan_and_tilt_steps.push_back({current_pan + (goal_pan - current_pan) / step_count * i,
                                    current_tilt + (goal_tilt - current_tilt) / step_count * i});
    }

    // Check if we have collisions on our path
    for (int i = 0; i < step_count; i++) {
      if (check_head_collision(pan_and_tilt_steps[i].first, pan_and_tilt_steps[i].second)) {
        // If we have a collision, try to move the head to an alternative position
        // The new position looks 10 degrees further up and is less likely to have a collision with the body
        // Also increase the depth of the recursion as this is a new attempt to move the head to the goal position
        return avoid_collision_on_path(goal_pan, goal_tilt + 10 * DEG_TO_RAD, current_pan, current_tilt, pan_speed,
                                       tilt_speed, max_depth, depth + 1);
      }
    }

    // We do not have any collisions on our path, so we can move the head to the goal position
    move_head_to_position_with_speed_adjustment(goal_pan, goal_tilt, current_pan, current_tilt, pan_speed, tilt_speed);
    return true;
  }

  /**
   * @brief Checks if the head collides with the body at a given pan and tilt position
   */
  bool check_head_collision(double pan, double tilt) {
    collision_state_->setJointPositions("HeadPan", &pan);
    collision_state_->setJointPositions("HeadTilt", &tilt);
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
    planning_scene_->checkCollision(req, res, *collision_state_, acm);
    return res.collision;
  }

  /**
   * @brief Move the head to the target position but adjust the speed of the joints so both reach the goal at the same
   * time
   */
  void move_head_to_position_with_speed_adjustment(double goal_pan, double goal_tilt, double current_pan,
                                                   double current_tilt, double pan_speed, double tilt_speed) {
    // Calculate the delta between the current and the goal positions
    double delta_pan = std::abs(goal_pan - current_pan);
    double delta_tilt = std::abs(goal_tilt - current_tilt);
    // Check which axis has to move further and adjust the speed of the other axis so both reach the goal at the same
    // time
    if (delta_pan > delta_tilt) {
      // Slow down the tilt axis to match the time it takes for the pan axis to reach the goal
      tilt_speed = std::min(tilt_speed, calculate_lower_speed(delta_pan, delta_tilt, pan_speed));
    } else {
      // Slow down the pan axis to match the time it takes for the tilt axis to reach the goal
      pan_speed = std::min(pan_speed, calculate_lower_speed(delta_tilt, delta_pan, tilt_speed));
    }

    // Send the motor goals including the position, speed and acceleration
    bitbots_msgs::msg::JointCommand pos_msg;
    pos_msg.header.stamp = rclcpp::Clock().now();
    pos_msg.joint_names = {"HeadPan", "HeadTilt"};
    pos_msg.positions = {goal_pan, goal_tilt};
    pos_msg.velocities = {pan_speed, tilt_speed};
    pos_msg.accelerations = {params_.max_acceleration_pan, params_.max_acceleration_pan};
    pos_msg.max_currents = {-1, -1};

    position_publisher_->publish(pos_msg);
  }

  /**
   * @brief Returns the current position of the head motors
   */
  std::pair<double, double> get_head_position() {
    double head_pan = 0.0;
    double head_tilt = 0.0;

    // Iterate over all joints and find the head pan and tilt joints
    for (size_t i = 0; i < current_joint_state_->name.size(); i++) {
      if (current_joint_state_->name[i] == "HeadPan") {
        head_pan = current_joint_state_->position[i];
      } else if (current_joint_state_->name[i] == "HeadTilt") {
        head_tilt = current_joint_state_->position[i];
      }
    }
    return {head_pan, head_tilt};
  }

  /**
   * @brief Converts a scanline number to a tilt angle
   */
  double lineAngle(int line, int line_count, double min_angle, double max_angle) {
    // Get the angular delta that is covered by the scanlines in the tilt axis
    double delta = std::abs(max_angle - min_angle);
    // Calculate the angular step size between two scanlines
    double steps = delta / (line_count - 1);
    // Calculate the tilt angle of the given scanline
    return steps * line + min_angle;
  }

  /**
   * @brief Performs a linear interpolation between the min and max pan values and returns the interpolated steps
   */
  std::vector<std::pair<double, double>> interpolatedSteps(int steps, double tilt, double min_pan, double max_pan) {
    // Handle edge case where we do not need to interpolate
    if (steps == 0) {
      return {};
    }
    // Add one to the step count as we need to include the min and max pan values
    steps += 1;
    // Create a vector that stores the interpolated steps
    std::vector<std::pair<double, double>> output_points;
    // Calculate the delta between the min and max pan values
    double delta = std::abs(max_pan - min_pan);
    // Calculate the step size between two interpolated steps
    double step_size = delta / steps;
    // Iterate over all steps and calculate the interpolated pan values
    for (int i = 1; i <= steps; i++) {
      double pan = min_pan + step_size * i;
      output_points.emplace_back(pan, tilt);
    }
    return output_points;
  }

  /**
   * @brief Generates a parameterized search pattern
   */
  std::vector<std::pair<double, double>> generatePattern(
      int line_count, double max_horizontal_angle_left, double max_horizontal_angle_right, double max_vertical_angle_up,
      double max_vertical_angle_down,
      int reduce_last_scanline = 0.2,  // TODO: needs to be changed to 1
      int interpolation_steps = 0) {
    // Store the keyframes of the search pattern
    std::vector<std::pair<double, double>> keyframes;
    // Store the state of the generation process
    bool down_direction = false;        // true = down, false = up
    bool right_side = false;            // true = right, false = left
    const bool right_direction = true;  // true = moving right, false = moving left
    int line = line_count - 1;
    // Calculate the number of iterations that are needed to generate the search pattern
    int iterations = std::max(line_count * 4 - 4, 2);
    // Iterate over all iterations and generate the search pattern
    for (int i = 0; i < iterations; i++) {
      // Get the maximum pan values (left and right) for the current pan position
      // Select the relevant one based on the current side we are on
      double current_pan;
      if (right_side) {
        current_pan = max_horizontal_angle_right;
      } else {
        current_pan = max_horizontal_angle_left;
      }

      // Get the current tilt angle based on the current line we are on
      double current_tilt = lineAngle(line, line_count, max_vertical_angle_down, max_vertical_angle_up);

      // Store the keyframe
      keyframes.push_back({current_pan, current_tilt});

      // Check if we move horizontally or vertically in the pattern
      if (right_side != right_direction) {
        // We move horizontally, so we might need to interpolate between the current and the next keyframe
        std::vector<std::pair<double, double>> interpolated_points =
            interpolatedSteps(interpolation_steps, current_tilt, max_horizontal_angle_right, max_horizontal_angle_left);
        // Reverse the order of the interpolated points if we are moving to the right
        if (right_direction) {
          std::reverse(interpolated_points.begin(), interpolated_points.end());
        }
        // Add the interpolated points to the keyframes
        keyframes.insert(keyframes.end(), interpolated_points.begin(), interpolated_points.end());
        // Change the direction we are moving in
        right_side = right_direction;

      } else {
        // Change the horizontal direction we are moving in
        right_side = !right_direction;
        // Check if we reached the top or bottom of the pattern and need to change the direction
        if (0 <= line && line <= line_count - 1) {
          down_direction = !down_direction;
        }
        // Change the line we are on based on the direction we are moving in
        if (down_direction) {
          line -= 1;
        } else {
          line += 1;
        }
      }
    }

    // Reduce the last scanline by a given factor
    for (auto& keyframe : keyframes) {
      if (keyframe.second == max_vertical_angle_down) {
        keyframe = {keyframe.first * reduce_last_scanline, max_vertical_angle_down};
      }
    }
    return keyframes;
  }

  /**
   * @brief Calculates the motor goals that are needed to look at a given point using the inverse kinematics
   */
  std::pair<double, double> get_motor_goals_from_point(geometry_msgs::msg::Point point) {
    // Create a new IK options object
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.return_approximate_solution = true;
    ik_options.replace = true;

    // Create a new look at goal and set the target point as the position the camera link should look at
    ik_options.goals.emplace_back(new bio_ik::LookAtGoal("camera", {1.0, 0.0, 0.0}, {point.x, point.y, point.z}));

    // Get the joint model group for the head
    auto joint_model_group = robot_model_->getJointModelGroup("Head");

    // Try to calculate the inverse kinematics
    double timeout_seconds = 1.0;
    bool success = robot_state_->setFromIK(joint_model_group, EigenSTL::vector_Isometry3d(), std::vector<std::string>(),
                                           timeout_seconds, moveit::core::GroupStateValidityCallbackFn(), ik_options);
    robot_state_->update();

    // Get the solution from the IK response
    bio_ik_msgs::msg::IKResponse response;
    moveit::core::robotStateToRobotStateMsg(*robot_state_, response.solution);
    response.solution_fitness = ik_options.solution_fitness;
    // Return the motor goals if the IK was successful
    if (success) {
      return {response.solution.joint_state.position[0], response.solution.joint_state.position[1]};
    } else {
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                   "BioIK failed with error code: " << response.error_code.val);
      return {0.0, 0.0};
    }
  }

  /**
   * @brief Looks at a given point and returns true if the goal position was reached
   */
  bool look_at(geometry_msgs::msg::PointStamped point, double min_pan_delta = 0.02, double min_tilt_delta = 0.02) {
    try {
      // Transform the point into the planning frame
      geometry_msgs::msg::PointStamped new_point =
          tf_buffer_->transform(point, planning_scene_->getPlanningFrame(), tf2::durationFromSec(0.9));

      // Get the motor goals that are needed to look at the point from the inverse kinematics
      std::pair<double, double> pan_tilt = get_motor_goals_from_point(new_point.point);
      // Get the current head position
      std::pair<double, double> current_pan_tilt = get_head_position();

      // Check if we reached the goal position
      if (std::abs(pan_tilt.first - current_pan_tilt.first) > min_pan_delta ||
          std::abs(pan_tilt.second - current_pan_tilt.second) > min_tilt_delta) {
        // Send the motor goals to the head motors
        send_motor_goals(pan_tilt.first, pan_tilt.second, true, params_.look_at.pan_speed, params_.look_at.tilt_speed);
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
   * @brief Returns the index of the pattern keypoint that is closest to the current head position
   *
   * @param pattern The search pattern
   * @param pan The current pan position
   * @param tilt The current tilt position
   * @return int The index of the pattern keypoint that is closest to the current head position
   */
  int get_near_pattern_position(std::vector<std::pair<double, double>> pattern, double pan, double tilt) {
    // Store the index and distance of the closest keypoint
    std::pair<double, int> min_distance_point = {10000.0, -1};
    // Iterate over all keypoints and calculate the distance to the current head position
    for (size_t i = 0; i < pattern.size(); i++) {
      // Calculate the cartesian distance between the current head position and the keypoint
      double distance = std::sqrt(std::pow(pattern[i].first - pan, 2) + std::pow(pattern[i].second - tilt, 2));
      // Check if the distance is smaller than the current minimum distance
      // and if so, update the minimum distance accordingly
      if (distance < min_distance_point.first) {
        min_distance_point.first = distance;
        min_distance_point.second = i;
      }
    }
    // Return the index of the closest keypoint
    return min_distance_point.second;
  }

  /**
   * @brief Performs the search pattern that is currently loaded
   */
  void perform_search_pattern() {
    // Do not perform the search pattern if it is empty
    if (pattern_.size() == 0) {
      return;
    }
    // Wrap the index that points to the current keypoint around if necessary
    index_ = index_ % int(pattern_.size());
    // Get the current keypoint and convert it to radians
    double head_pan = pattern_[index_].first * DEG_TO_RAD;
    double head_tilt = pattern_[index_].second * DEG_TO_RAD;
    // Get the current head position
    auto [current_head_pan, current_head_tilt] = get_head_position();

    // Send the motor goals to the head motors
    bool success =
        send_motor_goals(head_pan, head_tilt, false, pan_speed_, tilt_speed_, current_head_pan, current_head_tilt);

    if (success) {
      // Check if we reached the current keypoint and if so, increase the index
      double distance_to_goal =
          std::sqrt(std::pow(head_pan - current_head_pan, 2) + std::pow(head_tilt - current_head_tilt, 2));
      if (distance_to_goal <= threshold_) {
        index_++;
      }
    } else {
      // Skip the keypoint if we were not able to reach it (e.g. due to an unresolvable collision)
      index_++;
    }
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
        case bitbots_msgs::msg::HeadMode::SEARCH_BALL:
          pan_speed_ = params_.search_pattern.pan_speed;
          tilt_speed_ = params_.search_pattern.tilt_speed;
          pattern_ = generatePattern(params_.search_pattern.scan_lines, params_.search_pattern.pan_max[0],
                                     params_.search_pattern.pan_max[1], params_.search_pattern.tilt_max[0],
                                     params_.search_pattern.tilt_max[1], params_.search_pattern.reduce_last_scanline);
          break;
        case bitbots_msgs::msg::HeadMode::SEARCH_BALL_PENALTY:
          pan_speed_ = params_.search_pattern_penalty.pan_speed;
          tilt_speed_ = params_.search_pattern_penalty.tilt_speed;
          pattern_ =
              generatePattern(params_.search_pattern_penalty.scan_lines, params_.search_pattern_penalty.pan_max[0],
                              params_.search_pattern_penalty.pan_max[1], params_.search_pattern_penalty.tilt_max[0],
                              params_.search_pattern_penalty.tilt_max[1], params_.search_pattern.reduce_last_scanline);
          break;

        case bitbots_msgs::msg::HeadMode::SEARCH_FIELD_FEATURES:
          pan_speed_ = params_.search_pattern_field_features.pan_speed;
          tilt_speed_ = params_.search_pattern_field_features.tilt_speed;
          pattern_ = generatePattern(
              params_.search_pattern_field_features.scan_lines, params_.search_pattern_field_features.pan_max[0],
              params_.search_pattern_field_features.pan_max[1], params_.search_pattern_field_features.tilt_max[0],
              params_.search_pattern_field_features.tilt_max[1], params_.search_pattern.reduce_last_scanline);
          break;

        case bitbots_msgs::msg::HeadMode::SEARCH_FRONT:
          pan_speed_ = params_.front_search_pattern.pan_speed;
          tilt_speed_ = params_.front_search_pattern.tilt_speed;
          pattern_ =
              generatePattern(params_.front_search_pattern.scan_lines, params_.front_search_pattern.pan_max[0],
                              params_.front_search_pattern.pan_max[1], params_.front_search_pattern.tilt_max[0],
                              params_.front_search_pattern.tilt_max[1], params_.search_pattern.reduce_last_scanline);
          break;

        case bitbots_msgs::msg::HeadMode::LOOK_FORWARD:
          pan_speed_ = params_.look_forward.pan_speed;
          tilt_speed_ = params_.look_forward.tilt_speed;
          pattern_ = generatePattern(params_.look_forward.scan_lines, params_.look_forward.pan_max[0],
                                     params_.look_forward.pan_max[1], params_.look_forward.tilt_max[0],
                                     params_.look_forward.tilt_max[1], params_.search_pattern.reduce_last_scanline);
          break;

        default:
          return;
      }

      // Get the current head position
      std::pair<double, double> head_position = get_head_position();

      // Select the closest keypoint in the search pattern as a starting point
      index_ = get_near_pattern_position(pattern_, head_position.first, head_position.second);
    }
    // Check if no look at action is running or if the head mode is DONT_MOVE
    // if this is not the case, perform the search pattern
    if (!action_running_ && curr_head_mode != bitbots_msgs::msg::HeadMode::DONT_MOVE)  // here DONT_MOVE
                                                                                       // is implemented
    {
      // Store the current head mode as the previous head mode to detect changes
      prev_head_mode_ = curr_head_mode;
      // Execute the search pattern
      perform_search_pattern();
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
