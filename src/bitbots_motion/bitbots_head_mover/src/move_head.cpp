#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <bitbots_head_mover/active_vision.hpp>
#include <bitbots_head_mover/active_vision_debug.hpp>
#include <bitbots_head_mover/head_parameters.hpp>
#include <bitbots_head_mover/head_trajectory.hpp>
#include <bitbots_head_mover/look_at.hpp>
#include <bitbots_head_mover/search_pattern.hpp>
#include <bitbots_head_mover/types.hpp>
#include <bitbots_msgs/action/look_at.hpp>
#include <bitbots_msgs/msg/head_mode.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <bitbots_msgs/msg/team_data.hpp>
#include <bitbots_utils/utils.hpp>
#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <soccer_vision_3d_msgs/msg/ball_array.hpp>
#include <soccer_vision_3d_msgs/msg/robot_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2/convert.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace move_head {

using bitbots_head_mover::HeadLimits;
using bitbots_head_mover::HeadPosition;
using bitbots_head_mover::HeadVelocity;

/// Angular step size used when checking a path for collisions.
constexpr double kCollisionCheckStep = M_PI / 180.0 * 3.0;
/// Pitch offset applied when retrying a colliding goal further up.
constexpr double kCollisionAvoidancePitchStep = M_PI / 180.0 * 10.0;

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
  // Retries fetching the field dimensions until the parameter blackboard answers
  rclcpp::TimerBase::SharedPtr field_dimension_retry_timer_;

  // Declare variable for the current search pattern
  std::vector<HeadPosition> pattern_;
  // Store previous head mode
  uint prev_head_mode_ = -1;

  // Duration of one full search pattern cycle (seconds)
  double cycle_time_ = 0.0;

  // Spline trajectory for search patterns
  bitbots_head_mover::SearchPatternTrajectory search_trajectory_;
  rclcpp::Time spline_start_time_;

  // World model state
  geometry_msgs::msg::PoseWithCovarianceStamped ball_position_;

  // Action server for the look at action
  rclcpp_action::Server<LookAtGoal>::SharedPtr action_server_;
  bool action_running_ = false;

  // Active vision planner and everything that feeds it
  bitbots_head_mover::ActiveVision active_vision_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
  rclcpp::Subscription<soccer_vision_3d_msgs::msg::BallArray>::SharedPtr balls_subscriber_;
  rclcpp::Subscription<soccer_vision_3d_msgs::msg::RobotArray>::SharedPtr robots_subscriber_;
  rclcpp::Subscription<bitbots_msgs::msg::TeamData>::SharedPtr team_data_subscriber_;

  // The detection callbacks do blocking tf lookups. They run on their own
  // callback group, spun by a second executor on a dedicated thread, so a lookup
  // that waits for a not-yet-available transform never stalls the main executor
  // and the search pattern timer it drives.
  rclcpp::CallbackGroup::SharedPtr detection_callback_group_;
  // Guards the state the detection callbacks write and the main loop reads: the
  // world model and the latest filtered ball. Held only around those accesses,
  // never across a tf lookup, so the two threads never serialize on the wait.
  std::mutex world_mutex_;

  // Debug publishers, only created when the debug output is enabled
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr coverage_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr joint_space_publisher_;
  bool debug_publishers_created_ = false;

  // The last position commanded by the active vision mode, which is what the
  // head holds on to while an input is missing
  std::optional<HeadPosition> active_vision_hold_position_;

 public:
  HeadMover() : node_(std::make_shared<rclcpp::Node>("head_mover")) {
    // The detection callbacks block on tf lookups, so they get their own callback
    // group that a second executor spins on a dedicated thread. auto_add=false
    // keeps the main executor from also picking it up.
    detection_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions detection_options;
    detection_options.callback_group = detection_callback_group_;

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

    // Initialize subscriber for the ball filter. It does a blocking tf lookup and
    // writes shared state, so it belongs on the detection callback group too.
    ball_filter_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "ball_position_relative_filtered", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          {
            std::lock_guard<std::mutex> lock(world_mutex_);
            // cppcheck-suppress useInitializationList
            ball_position_ = *msg;
          }
          handle_filtered_ball(*msg);
        },
        detection_options);

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

    setup_active_vision();

    // Initialize timer for main loop
    timer_ = rclcpp::create_timer(node_, node_->get_clock(), 50ms, [this] { behave(); });
  }

  /**
   * @brief Sets up the inputs of the active vision head mode
   */
  void setup_active_vision() {
    // The robot description is latched by the robot state publisher, so a
    // transient local subscription still receives it when we start later
    robot_description_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) { handle_robot_description(msg->data); });

    camera_info_subscriber_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/zed/zed_node/rgb/camera_info", 1, [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          if (!active_vision_.setCameraInfo(*msg)) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Received camera info without usable intrinsics, active vision stays disabled");
          }
        });

    // The detection callbacks do blocking tf lookups, so they run on the
    // dedicated callback group rather than the main executor.
    rclcpp::SubscriptionOptions detection_options;
    detection_options.callback_group = detection_callback_group_;

    balls_subscriber_ = node_->create_subscription<soccer_vision_3d_msgs::msg::BallArray>(
        "balls_relative", 1, [this](const soccer_vision_3d_msgs::msg::BallArray::SharedPtr msg) { handle_balls(*msg); },
        detection_options);

    robots_subscriber_ = node_->create_subscription<soccer_vision_3d_msgs::msg::RobotArray>(
        "robots_relative", 1,
        [this](const soccer_vision_3d_msgs::msg::RobotArray::SharedPtr msg) { handle_robots(*msg); },
        detection_options);

    team_data_subscriber_ = node_->create_subscription<bitbots_msgs::msg::TeamData>(
        "team_data", 10, [this](const bitbots_msgs::msg::TeamData::SharedPtr msg) { handle_team_data(*msg); },
        detection_options);

    apply_active_vision_parameters();

    // Try once at startup, and otherwise keep retrying until it works. Giving up
    // after a single attempt would disable the mode for the rest of the run
    // because of a startup race with the parameter blackboard.
    //
    // The retry lives on its own slow timer rather than in the control loop: the
    // parameter call blocks for up to a second, which would stall the 20 Hz loop
    // on every tick for as long as the blackboard is unreachable.
    if (!try_fetch_field_dimensions()) {
      field_dimension_retry_timer_ = rclcpp::create_timer(node_, node_->get_clock(), 2s, [this] {
        if (try_fetch_field_dimensions()) {
          field_dimension_retry_timer_->cancel();
        }
      });
    }
  }

  /**
   * @brief Pulls the field dimensions from the global parameter server
   *
   * They are not part of this node's schema, they live on the parameter
   * blackboard the way the localization reads them as well.
   */
  bool try_fetch_field_dimensions() {
    try {
      auto global_params = bitbots_utils::get_parameters_from_other_node(node_, "/parameter_blackboard",
                                                                         {"field.size.x", "field.size.y"}, 1s);
      bitbots_head_mover::FieldCoverageConfig coverage;
      coverage.field_length = global_params.at("field.size.x").as_double();
      coverage.field_width = global_params.at("field.size.y").as_double();

      // A degenerate field would build an empty or nonsensical coverage map,
      // which is worth saying out loud rather than quietly steering the head
      if (!(coverage.field_length > 0.0) || !(coverage.field_width > 0.0)) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                              "The parameter blackboard reports a field of %.2f x %.2f m, which is not usable",
                              coverage.field_length, coverage.field_width);
        return false;
      }

      coverage.margin = params_.active_vision.coverage.margin;
      coverage.cell_size = params_.active_vision.coverage.cell_size;
      coverage.half_life = params_.active_vision.coverage.half_life;
      active_vision_.setFieldCoverageConfig(coverage);
      RCLCPP_INFO(node_->get_logger(), "Active vision uses a %.2f x %.2f m field", coverage.field_length,
                  coverage.field_width);
      return true;
    } catch (const std::exception& ex) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                            "Could not get the field dimensions from the parameter blackboard, active vision cannot "
                            "run yet: %s",
                            ex.what());
      return false;
    }
  }

  /**
   * @brief Builds the head chain from a received robot description
   */
  void handle_robot_description(const std::string& urdf) {
    bitbots_head_mover::HeadChainConfig chain;
    chain.root_link = params_.active_vision.root_link;
    chain.tip_link = params_.active_vision.tip_link;

    if (!active_vision_.setRobotDescription(urdf, chain)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Could not build the head chain from '%s' to '%s', active vision stays disabled",
                   chain.root_link.c_str(), chain.tip_link.c_str());
      return;
    }
    RCLCPP_INFO(node_->get_logger(), "Built the head chain from '%s' to '%s'", chain.root_link.c_str(),
                chain.tip_link.c_str());

    // The extrinsic calibration is published as its own transform rather than
    // being part of the robot description, so it has to be composed onto the
    // chain's tip. It does not depend on the head position, so one lookup is
    // enough, but it is only available once that publisher is up.
    update_camera_calibration();
  }

  /**
   * @brief Looks up the extrinsic camera calibration and hands it to the planner
   */
  bool update_camera_calibration() {
    const std::string& uncalibrated = params_.active_vision.tip_link;
    const std::string& calibrated = params_.active_vision.calibrated_optical_frame;
    if (uncalibrated == calibrated) {
      return true;
    }

    try {
      const auto transform =
          tf_buffer_->lookupTransform(uncalibrated, calibrated, tf2::TimePointZero, tf2::durationFromSec(0.1));
      active_vision_.setCameraCalibration(tf2::transformToEigen(transform));
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "Could not look up the extrinsic camera calibration from '%s' to '%s': %s",
                           uncalibrated.c_str(), calibrated.c_str(), ex.what());
      return false;
    }
  }

  /**
   * @brief Pushes the current parameters into the active vision planner
   */
  void apply_active_vision_parameters() {
    const auto& config = params_.active_vision;

    bitbots_head_mover::SamplerConfig sampler;
    sampler.sample_count = static_cast<int>(config.sampling.sample_count);
    sampler.last_target_weight = config.sampling.last_target_weight;
    sampler.current_position_weight = config.sampling.current_position_weight;
    sampler.uniform_weight = config.sampling.uniform_weight;
    sampler.last_target_std = config.sampling.last_target_std;
    sampler.current_position_std = config.sampling.current_position_std;
    active_vision_.setSamplerConfig(sampler);

    bitbots_head_mover::HeadController controller;
    controller.approach_distance = config.controller.approach_distance;
    controller.control_period = config.controller.control_period;
    controller.max_velocity = {config.max_velocity_yaw, config.max_velocity_pitch};
    active_vision_.setController(controller);

    active_vision_.setHeadLimits(get_head_limits());
    active_vision_.setVisibilityWeighting({config.visibility.center_fraction, config.visibility.border_score});
    active_vision_.setCoverageDistanceHalfWeight(config.coverage.distance_half_weight);

    bitbots_head_mover::WorldModelConfig world;
    world.filtered_ball_timeout = config.timeouts.filtered_ball;
    world.raw_ball_timeout = config.timeouts.raw_ball;
    world.team_ball_timeout = config.timeouts.team_ball;
    world.robot_timeout = config.timeouts.robot;
    world.covariance_half_weight = config.covariance_half_weight;
    {
      // Applied from the main loop on a parameter change, while the detection
      // thread may be writing detections into the same world model
      std::lock_guard<std::mutex> lock(world_mutex_);
      active_vision_.setWorldModelConfig(world);
    }

    bitbots_head_mover::ScoringWeights weights;
    weights.filtered_ball = config.weights.filtered_ball;
    weights.raw_balls = config.weights.raw_balls;
    weights.team_ball = config.weights.team_ball;
    weights.field_coverage = config.weights.field_coverage;
    weights.robots = config.weights.robots;
    weights.smoothness = config.weights.smoothness;
    active_vision_.setScoringWeights(weights);
  }

  /**
   * @brief Returns the head joint limits as defined in the parameters
   */
  HeadLimits get_head_limits() const {
    return {{params_.max_yaw[0], params_.max_yaw[1]}, {params_.max_pitch[0], params_.max_pitch[1]}};
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
    geometry_msgs::msg::PointStamped rel_head_yaw_point;
    try {
      rel_head_yaw_point = tf_buffer_->transform(goal->look_at_position, "head_yaw_link", tf2::durationFromSec(0.9));
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "Could not transform goal point: %s", ex.what());
      return rclcpp_action::GoalResponse::REJECT;
    }

    geometry_msgs::msg::PointStamped rel_head_pitch_point;
    try {
      rel_head_pitch_point =
          tf_buffer_->transform(goal->look_at_position, "head_pitch_link", tf2::durationFromSec(0.9));
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "Could not transform goal point: %s", ex.what());
      return rclcpp_action::GoalResponse::REJECT;
    }

    // The goal is computed relative to where the head is, so an unknown head
    // position means the goal cannot be judged and the action has to be rejected
    const auto current = require_head_position("a look at goal request");
    if (!current) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Get the motor goals that are needed to look at the point
    HeadPosition goal_position =
        bitbots_head_mover::motorGoalsFromPoint(rel_head_yaw_point.point, rel_head_pitch_point.point, *current);

    // Check whether the action goal is valid and can be executed. The head
    // limits are checked twice by the original implementation, once as a
    // collision check and once directly, which is equivalent to a single check.
    if (action_running_ || !get_head_limits().contains(goal_position)) {
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
   * @brief Send the goal positions to the head motors, but resolve collisions with the body if necessary.
   *
   */
  bool send_motor_goals(HeadPosition goal, bool resolve_collision, const HeadVelocity& speeds = {1.5, 1.5},
                        const HeadPosition& current = {}, bool clip = true) {
    // Debug log the target yaw and pitch position
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "target yaw/pitch: " << goal.yaw << "/" << goal.pitch);

    // Clip the target yaw and pitch position at the maximum yaw and pitch values as defined in the parameters
    if (clip) {
      goal = get_head_limits().clamp(goal);
    }

    // Resolve collisions if necessary
    if (resolve_collision) {
      // Call behavior that resolves collisions and might change the target yaw and pitch position
      bool success = avoid_collision_on_path(goal, current, speeds);
      // Report error message of we were not able to move to an alternative collision free position
      if (!success) {
        RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                     "Unable to resolve head collision");
      }
      return success;
    } else {
      // Move the head to the target position but adjust the speed of the joints so both reach the goal at the same time
      move_head_to_position_with_speed_adjustment(goal, current, speeds);
      return true;
    }
  }

  /**
   * @brief Tries to move the head to the target position but resolves collisions with the body if necessary.
   *
   */
  bool avoid_collision_on_path(HeadPosition goal, const HeadPosition& current, const HeadVelocity& speeds,
                               int max_depth = 4, int depth = 0) {
    // Check if we reached the maximum depth of the recursion and if so, return false
    if (depth > max_depth) {
      return false;
    }

    // Calculate the distance between the current and the goal position
    double distance = std::sqrt(std::pow(goal.yaw - current.yaw, 2) + std::pow(goal.pitch - current.pitch, 2));

    // Calculate the number of steps we need to take to reach the goal position
    int step_count = distance / kCollisionCheckStep;

    // Check if we have collisions on our path by performing linear interpolation
    // between the current and the goal position
    const HeadLimits limits = get_head_limits();
    for (int i = 0; i < step_count; i++) {
      HeadPosition step = {current.yaw + (goal.yaw - current.yaw) / step_count * i,
                           current.pitch + (goal.pitch - current.pitch) / step_count * i};
      if (!limits.contains(step)) {
        // If we have a collision, try to move the head to an alternative position
        // The new position looks further up and is less likely to have a collision with the body
        // Also increase the depth of the recursion as this is a new attempt to move the head to the goal position
        goal.pitch += kCollisionAvoidancePitchStep;
        return avoid_collision_on_path(goal, current, speeds, max_depth, depth + 1);
      }
    }

    // We do not have any collisions on our path, so we can move the head to the goal position
    move_head_to_position_with_speed_adjustment(goal, current, speeds);
    return true;
  }

  /**
   * @brief Move the head to the target position but adjust the speed of the joints so both reach the goal at the same
   * time
   */
  void move_head_to_position_with_speed_adjustment(const HeadPosition& goal, const HeadPosition& current,
                                                   const HeadVelocity& speeds) {
    publish_motor_goals(goal, bitbots_head_mover::adjustSpeeds(goal, current, speeds));
  }

  /**
   * @brief Publishes the given head position and joint speeds as motor goals
   */
  void publish_motor_goals(const HeadPosition& goal, const HeadVelocity& speeds) {
    // Send the motor goals including the position, speed and acceleration
    bitbots_msgs::msg::JointCommand pos_msg;
    pos_msg.header.stamp = node_->get_clock()->now();
    pos_msg.joint_names = {"head_yaw_joint", "head_pitch_joint"};
    pos_msg.positions = {goal.yaw, goal.pitch};
    pos_msg.velocities = {speeds.yaw, speeds.pitch};
    pos_msg.accelerations = {params_.max_acceleration_yaw, params_.max_acceleration_pitch};
    pos_msg.max_torques = {10, 10};

    position_publisher_->publish(pos_msg);
  }

  /**
   * @brief Returns the current position of the head motors
   *
   * Returns nothing if the joint state does not carry both head joints. Falling
   * back to zero here would be indistinguishable from the head genuinely looking
   * straight ahead, and every goal computed relative to it would be wrong by
   * however far the head actually is from center.
   */
  std::optional<HeadPosition> get_head_position() const {
    HeadPosition position;
    bool found_yaw = false;
    bool found_pitch = false;
    // Iterate over all joints and find the head yaw and pitch joints
    for (size_t i = 0; i < current_joint_state_->name.size(); i++) {
      const bool is_yaw = current_joint_state_->name[i] == "head_yaw_joint";
      const bool is_pitch = current_joint_state_->name[i] == "head_pitch_joint";
      if (!is_yaw && !is_pitch) {
        continue;
      }
      // The parallel arrays are only required to be as long as the caller filled
      // them, so a name without a matching position must not be indexed
      if (i >= current_joint_state_->position.size()) {
        return std::nullopt;
      }
      if (is_yaw) {
        position.yaw = current_joint_state_->position[i];
        found_yaw = true;
      } else {
        position.pitch = current_joint_state_->position[i];
        found_pitch = true;
      }
    }
    if (!found_yaw || !found_pitch) {
      return std::nullopt;
    }
    return position;
  }

  /**
   * @brief Returns the head position, reporting loudly if it is unavailable
   *
   * Used by the paths that cannot proceed without knowing where the head is.
   */
  std::optional<HeadPosition> require_head_position(const char* what) const {
    const auto position = get_head_position();
    if (!position) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "Joint states carry no usable head_yaw_joint and head_pitch_joint position, skipping %s",
                            what);
    }
    return position;
  }

  /**
   * @brief Looks at a given point and returns true if the goal position was reached
   */
  bool look_at(geometry_msgs::msg::PointStamped point, double min_yaw_delta = 0.02, double min_pitch_delta = 0.02) {
    try {
      // Transform the point into the planning frame
      geometry_msgs::msg::PointStamped rel_head_yaw_point =
          tf_buffer_->transform(point, "head_yaw_link", tf2::durationFromSec(0.9));

      geometry_msgs::msg::PointStamped rel_head_pitch_point =
          tf_buffer_->transform(point, "head_pitch_link", tf2::durationFromSec(0.9));

      // Get the current head position
      const auto maybe_current = require_head_position("a look at update");
      if (!maybe_current) {
        return false;
      }
      const HeadPosition current = *maybe_current;

      // Get the motor goals that are needed to look at the point from the inverse kinematics
      HeadPosition goal =
          bitbots_head_mover::motorGoalsFromPoint(rel_head_yaw_point.point, rel_head_pitch_point.point, current);

      // Check if we reached the goal position
      if (std::abs(goal.yaw - current.yaw) > min_yaw_delta || std::abs(goal.pitch - current.pitch) > min_pitch_delta) {
        // Send the motor goals to the head motors
        send_motor_goals(goal, true, {params_.look_at.yaw_speed, params_.look_at.pitch_speed});
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
   * @brief Builds an open-loop trajectory from the current search pattern, starting at the
   * current head position.
   */
  void build_spline_trajectory() {
    // The trajectory transitions out of the current head position, so building
    // it without knowing that position would start the pattern with a jump from
    // wherever the head happens to be to the assumed center
    const auto start = require_head_position("building a search pattern trajectory");
    if (!start) {
      search_trajectory_ = {};
      return;
    }
    search_trajectory_ =
        bitbots_head_mover::buildSearchPatternTrajectory(pattern_, cycle_time_, *start, params_.transition_speed);
    if (!search_trajectory_.valid()) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                            "Could not build a search pattern trajectory from %zu keyframes with a cycle time of %.2fs",
                            pattern_.size(), cycle_time_);
      return;
    }
    spline_start_time_ = node_->now();
  }

  /**
   * @brief Evaluates the pre-built spline trajectory at the current time and publishes
   * the resulting joint position and velocity as open-loop motor goals.
   */
  void perform_search_pattern() {
    if (!search_trajectory_.valid()) {
      return;
    }

    // Play the transition from the previous head position once, then loop only over the cyclic part of the trajectory
    double t = search_trajectory_.phase((node_->now() - spline_start_time_).seconds());

    HeadPosition goal = get_head_limits().clamp(search_trajectory_.trajectory.position(t));
    HeadVelocity velocity = search_trajectory_.trajectory.velocity(t);

    // The motor goals carry joint speeds, so the direction of travel is dropped here
    publish_motor_goals(goal, {std::abs(velocity.yaw), std::abs(velocity.pitch)});
  }

  /**
   * @brief Looks up the transform that brings a message's frame into the map frame
   *
   * A detection array shares one header, so the transform is looked up once per
   * message and then applied to every point in it. Transforming each point on
   * its own would repeat the same buffer lookup for every ball or robot in the
   * message.
   */
  bool lookup_to_map(const std_msgs::msg::Header& header, Eigen::Isometry3d& transform) {
    try {
      // This runs in the detection callbacks, which are spun on their own executor
      // thread, so waiting a short while for a transform that is still on its way
      // does not stall the main loop timer. The wait is not self defeating either:
      // the transform listener fills the buffer from its own dedicated thread, so
      // the awaited transform can still arrive while we block here.
      transform = tf2::transformToEigen(tf_buffer_->lookupTransform(params_.active_vision.map_frame, header.frame_id,
                                                                    header.stamp, tf2::durationFromSec(0.1)));
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "Could not transform a detection from '%s' into '%s': %s", header.frame_id.c_str(),
                           params_.active_vision.map_frame.c_str(), ex.what());
      return false;
    }
  }

  /**
   * @brief Applies a transform to a point from a message
   */
  static Eigen::Vector3d transform_point(const Eigen::Isometry3d& transform, const geometry_msgs::msg::Point& point) {
    return transform * Eigen::Vector3d(point.x, point.y, point.z);
  }

  /**
   * @brief Stores the filtered ball estimate in the map frame
   */
  void handle_filtered_ball(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
    Eigen::Isometry3d to_map;
    if (!lookup_to_map(msg.header, to_map)) {
      return;
    }
    const Eigen::Vector3d position = transform_point(to_map, msg.pose.pose.position);
    // The two planar variances are what says how well the filter knows the ball,
    // taking the larger of them treats an estimate that is uncertain in any
    // direction as uncertain
    const double covariance = std::max(msg.pose.covariance[0], msg.pose.covariance[7]);
    // Guard only the world write, never the lookup above, so the main loop is not
    // serialized behind a blocking transform wait
    std::lock_guard<std::mutex> lock(world_mutex_);
    active_vision_.world().setFilteredBall(position, covariance, rclcpp::Time(msg.header.stamp).seconds());
  }

  /**
   * @brief Stores the raw ball detections in the map frame
   */
  void handle_balls(const soccer_vision_3d_msgs::msg::BallArray& msg) {
    Eigen::Isometry3d to_map;
    if (!lookup_to_map(msg.header, to_map)) {
      return;
    }

    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    std::vector<bitbots_head_mover::TimedTarget> balls;
    balls.reserve(msg.balls.size());
    for (const auto& ball : msg.balls) {
      // The detection confidence is what the raw detections are weighted by,
      // they carry no covariance of their own
      balls.push_back({transform_point(to_map, ball.center), static_cast<double>(ball.confidence.confidence), stamp});
    }
    std::lock_guard<std::mutex> lock(world_mutex_);
    active_vision_.world().setRawBalls(std::move(balls));
  }

  /**
   * @brief Stores the robot detections in the map frame
   */
  void handle_robots(const soccer_vision_3d_msgs::msg::RobotArray& msg) {
    Eigen::Isometry3d to_map;
    if (!lookup_to_map(msg.header, to_map)) {
      return;
    }

    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    std::vector<bitbots_head_mover::TimedTarget> robots;
    robots.reserve(msg.robots.size());
    for (const auto& robot : msg.robots) {
      robots.push_back(
          {transform_point(to_map, robot.bb.center.position), static_cast<double>(robot.confidence.confidence), stamp});
    }
    std::lock_guard<std::mutex> lock(world_mutex_);
    active_vision_.world().setRobots(std::move(robots));
  }

  /**
   * @brief Stores the ball a teammate reports
   */
  void handle_team_data(const bitbots_msgs::msg::TeamData& msg) {
    // Teammates report in the map frame already, and a penalized robot's ball is
    // not worth turning the head for
    if (msg.state == bitbots_msgs::msg::TeamData::STATE_PENALIZED) {
      return;
    }
    const auto& ball = msg.ball_absolute;
    // The covariance is a full 6x6 matrix, of which the two planar variances are
    // what tells us how well the teammate knows where the ball is
    const double covariance = std::max(ball.covariance[0], ball.covariance[7]);
    std::lock_guard<std::mutex> lock(world_mutex_);
    active_vision_.world().setTeamBall(
        msg.robot_id, Eigen::Vector3d(ball.pose.position.x, ball.pose.position.y, ball.pose.position.z), covariance,
        rclcpp::Time(msg.header.stamp).seconds());
  }

  /**
   * @brief Creates the debug publishers on demand
   */
  void ensure_debug_publishers() {
    if (debug_publishers_created_) {
      return;
    }
    coverage_publisher_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>("debug/active_vision/field_coverage", 1);
    candidate_publisher_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>("debug/active_vision/candidates", 1);
    joint_space_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("debug/active_vision/joint_space", 1);
    debug_publishers_created_ = true;
  }

  /**
   * @brief Publishes the coverage grid, the candidate markers and the joint space plot
   */
  void publish_active_vision_debug(const bitbots_head_mover::ActiveVisionResult& result,
                                   const Eigen::Isometry3d& robot_pose) {
    ensure_debug_publishers();

    const auto stamp = node_->get_clock()->now();
    const std::string& map_frame = params_.active_vision.map_frame;

    coverage_publisher_->publish(bitbots_head_mover::coverageGrid(active_vision_.coverage(), map_frame, stamp));
    candidate_publisher_->publish(
        bitbots_head_mover::candidateMarkers(result, active_vision_, robot_pose, map_frame, stamp));

    const cv::Mat plot = bitbots_head_mover::jointSpaceDebugImage(
        result, active_vision_.headLimits(), static_cast<int>(params_.active_vision.debug.image_size));

    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = map_frame;
    joint_space_publisher_->publish(*cv_bridge::CvImage(header, "bgr8", plot).toImageMsg());
  }

  /**
   * @brief Runs one cycle of the active vision head mode
   *
   * Holds the last commanded position when an input is missing, so a missing
   * transform or a camera that has not come up yet is visible as a head that
   * stops rather than one that silently falls back to a sweep.
   */
  void perform_active_vision() {
    // Inputs that can arrive late are retried by their own subscriptions and by
    // the field dimension retry timer, so this only has to report the wait
    const auto readiness = active_vision_.readiness();
    if (readiness != bitbots_head_mover::ActiveVisionReadiness::Ready) {
      hold_active_vision_position(bitbots_head_mover::describe(readiness));
      return;
    }

    // The calibration may only become available after the description did
    if (!active_vision_.kinematics().hasCameraCalibration() && !update_camera_calibration()) {
      hold_active_vision_position("the extrinsic camera calibration is unavailable");
      return;
    }

    // Where the robot stands on the field, including the orientation the IMU
    // contributes, which is what decides what the head can actually see
    Eigen::Isometry3d robot_pose;
    try {
      robot_pose = tf2::transformToEigen(tf_buffer_->lookupTransform(params_.active_vision.map_frame,
                                                                     params_.active_vision.root_link,
                                                                     tf2::TimePointZero, tf2::durationFromSec(0.1)));
    } catch (const tf2::TransformException& ex) {
      hold_active_vision_position(ex.what());
      return;
    }

    const auto head_position = require_head_position("an active vision cycle");
    if (!head_position) {
      hold_active_vision_position("the current head position is unknown");
      return;
    }

    bitbots_head_mover::ActiveVisionInput input;
    input.head_position = *head_position;
    input.robot_pose = robot_pose;
    input.now = node_->now().seconds();

    bitbots_head_mover::ActiveVisionResult result;
    {
      // The detection thread writes the world model concurrently; hold it steady
      // for the duration of the planning cycle that reads and prunes it
      std::lock_guard<std::mutex> lock(world_mutex_);
      result = active_vision_.plan(input);
    }
    if (!result.valid) {
      hold_active_vision_position(bitbots_head_mover::describe(result.failure));
      return;
    }

    // A partially unscorable candidate set still yields a decision, but it means
    // the planner chose from fewer options than it sampled
    if (result.unscorable_candidates > 0) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "Discarded %zu of %zu head target candidates because they could not be scored",
                           result.unscorable_candidates, result.candidates.size());
    }

    active_vision_hold_position_ = result.position;
    // The planner's proportional controller already produced the setpoint one
    // step towards the selected target and the speed to reach it; the motor goals
    // carry joint speeds, so the direction of travel is dropped
    publish_motor_goals(result.position, {std::abs(result.velocity.yaw), std::abs(result.velocity.pitch)});

    if (params_.active_vision.debug.enabled) {
      // The debug markers rebuild a scorer over the world model, so this read has
      // to be guarded against the detection thread as well
      std::lock_guard<std::mutex> lock(world_mutex_);
      publish_active_vision_debug(result, robot_pose);
    }
  }

  /**
   * @brief Keeps the head where it is because the active vision mode cannot plan
   */
  void hold_active_vision_position(const std::string& reason) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Active vision is holding position: %s",
                         reason.c_str());
    // Publishing nothing is what holds the head: the motors keep the last goal
    // they were given. Sending a goal with a zero speed instead would depend on
    // how the hardware interface reads a zero velocity, which is exactly the
    // ambiguity this mode should not rely on. This mirrors how DONT_MOVE works.
  }

  /**
   * @brief Callback for the ticks of the main loop
   */
  void behave() {
    // Get the current head mode
    uint curr_head_mode = head_mode_;

    // Pull the parameters from the parameter server
    if (param_listener_->is_old(params_)) {
      params_ = param_listener_->get_params();
      apply_active_vision_parameters();
    }

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

        case bitbots_msgs::msg::HeadMode::ACTIVE_VISION:
          // Entering active vision starts from wherever the head currently is,
          // there is no pattern to precompute
          active_vision_hold_position_.reset();
          break;
        case bitbots_msgs::msg::HeadMode::SEARCH_BALL_PENALTY:
          cycle_time_ = params_.search_patterns.search_ball_penalty.cycle_time;
          pattern_ =
              bitbots_head_mover::generatePattern(params_.search_patterns.search_ball_penalty.scan_lines,
                                                  params_.search_patterns.search_ball_penalty.yaw_max[0],
                                                  params_.search_patterns.search_ball_penalty.yaw_max[1],
                                                  params_.search_patterns.search_ball_penalty.pitch_max[0],
                                                  params_.search_patterns.search_ball_penalty.pitch_max[1],
                                                  params_.search_patterns.search_ball_penalty.reduce_last_scanline);
          break;

        case bitbots_msgs::msg::HeadMode::SEARCH_FIELD_FEATURES:
          cycle_time_ = params_.search_patterns.search_field_features.cycle_time;
          pattern_ =
              bitbots_head_mover::generatePattern(params_.search_patterns.search_field_features.scan_lines,
                                                  params_.search_patterns.search_field_features.yaw_max[0],
                                                  params_.search_patterns.search_field_features.yaw_max[1],
                                                  params_.search_patterns.search_field_features.pitch_max[0],
                                                  params_.search_patterns.search_field_features.pitch_max[1],
                                                  params_.search_patterns.search_field_features.reduce_last_scanline);
          break;

        case bitbots_msgs::msg::HeadMode::SEARCH_FRONT:
          cycle_time_ = params_.search_patterns.search_front.cycle_time;
          pattern_ = bitbots_head_mover::generatePattern(
              params_.search_patterns.search_front.scan_lines, params_.search_patterns.search_front.yaw_max[0],
              params_.search_patterns.search_front.yaw_max[1], params_.search_patterns.search_front.pitch_max[0],
              params_.search_patterns.search_front.pitch_max[1],
              params_.search_patterns.search_front.reduce_last_scanline);
          break;

        case bitbots_msgs::msg::HeadMode::LOOK_FORWARD:
          cycle_time_ = params_.search_patterns.look_forward.cycle_time;
          pattern_ = bitbots_head_mover::generatePattern(
              params_.search_patterns.look_forward.scan_lines, params_.search_patterns.look_forward.yaw_max[0],
              params_.search_patterns.look_forward.yaw_max[1], params_.search_patterns.look_forward.pitch_max[0],
              params_.search_patterns.look_forward.pitch_max[1],
              params_.search_patterns.look_forward.reduce_last_scanline);
          break;

        default:
          // An unknown mode means the sender and this node disagree about the
          // HeadMode message. Silently doing nothing would look exactly like a
          // head that was told to hold still.
          RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                "Received unknown head mode %u, ignoring it and keeping the previous mode",
                                curr_head_mode);
          return;
      }

      // Store the current head mode as the previous head mode to detect changes
      prev_head_mode_ = curr_head_mode;

      // Build the spline trajectory from the new pattern. It starts with a smooth transition from
      // the current head position, so we also rebuild it when we re-enter a search pattern from
      // e.g. ball tracking instead of continuing the old trajectory with an abrupt movement.
      if (curr_head_mode != bitbots_msgs::msg::HeadMode::TRACK_BALL &&
          curr_head_mode != bitbots_msgs::msg::HeadMode::DONT_MOVE &&
          curr_head_mode != bitbots_msgs::msg::HeadMode::ACTIVE_VISION) {
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
        // The filtered ball is written by the detection thread, so take a
        // consistent snapshot of it under the lock before using it
        geometry_msgs::msg::PoseWithCovarianceStamped ball_snapshot;
        {
          std::lock_guard<std::mutex> lock(world_mutex_);
          ball_snapshot = ball_position_;
        }
        // Convert the ball position to a PointStamped
        geometry_msgs::msg::PointStamped look_at_point;
        look_at_point.header = ball_snapshot.header;
        look_at_point.point = ball_snapshot.pose.pose.position;
        // Try to look at the ball
        look_at(look_at_point);
      } else if (curr_head_mode == bitbots_msgs::msg::HeadMode::ACTIVE_VISION) {
        // Decide where to look by sampling and scoring head trajectories
        perform_active_vision();
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

  /**
   * @brief The callback group carrying the blocking detection callbacks
   *
   * Spun by a second executor on its own thread so their tf waits do not stall
   * the main executor that drives the search pattern timer.
   */
  rclcpp::CallbackGroup::SharedPtr get_detection_callback_group() { return detection_callback_group_; }
};
}  // namespace move_head

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::experimental::executors::EventsExecutor exec;
  auto head_mover = std::make_shared<move_head::HeadMover>();
  exec.add_node(head_mover->get_node());

  // The detection callbacks block on tf lookups. Spinning them on a second
  // executor on a dedicated thread keeps that wait off the main executor, so the
  // search pattern timer keeps ticking at a steady rate. The detection callback
  // group was created with auto_add=false, so add_node above did not pick it up.
  rclcpp::executors::SingleThreadedExecutor detection_exec;
  detection_exec.add_callback_group(head_mover->get_detection_callback_group(),
                                    head_mover->get_node()->get_node_base_interface());
  std::thread detection_thread([&detection_exec]() { detection_exec.spin(); });

  exec.spin();

  detection_exec.cancel();
  detection_thread.join();
  rclcpp::shutdown();

  return 0;
}
