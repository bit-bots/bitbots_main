
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <iostream>
#include "head_parameters.hpp"
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

#include <humanoid_league_msgs/msg/head_mode.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <bio_ik/bio_ik.h>
#include <moveit/robot_state/conversions.h>
#include <tf2/convert.h>

#include <bio_ik_msgs/msg/ik_response.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/executors/events_executor/events_executor.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class HeadMover : public rclcpp::Node {

  //declare subscriber and publisher
  rclcpp::Subscription<humanoid_league_msgs::msg::HeadMode>::SharedPtr head_mode_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ball_subscriber_;

  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr position_publisher_;

  // declare tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  //declare variables
  humanoid_league_msgs::msg::HeadMode head_mode_;
  sensor_msgs::msg::JointState current_joint_state_;
  bitbots_msgs::msg::JointCommand pos_msg_;
  double DEG_TO_RAD = M_PI / 180;
  geometry_msgs::msg::PoseWithCovarianceStamped tf_precision_pose_;

  //declare robotmodel and planning scene
  robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningScenePtr planning_scene_;

  //declare world model variables
  geometry_msgs::msg::PointStamped ball_; //help: in python its from tf2_geometry msgs
  geometry_msgs::msg::PointStamped ball_odom_; // same her
  geometry_msgs::msg::PointStamped ball_map_;
  geometry_msgs::msg::PointStamped ball_teammate_;

  //declare some more params
  std::string odom_frame_ = "odom";
  std::string map_frame_ = "map";
  std::string head_tf_frame_ = "base_link";

  //declare params
  move_head::Params params_;
  double pan_speed_;
  double tilt_speed_;

  std::shared_ptr<rclcpp::executors::EventsExecutor> exec_;
  std::thread t_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::pair<double, double>> pattern_;
  uint prev_head_mode;

    double threshold_; 
  int index_;
    double pan_speed;
  double tilt_speed;

 public:
  HeadMover()
      : Node("head_mover", rclcpp::NodeOptions().allow_undeclared_parameters(true)) {

    timer_ = this->create_wall_timer(
        500ms, std::bind(&HeadMover::behave, this));
    position_publisher_ = this->create_publisher<bitbots_msgs::msg::JointCommand>("head_motor_goals", 10);
    head_mode_subscriber_ =
        this->create_subscription<humanoid_league_msgs::msg::HeadMode>( // here we want to call world_model.ball_filtered_callback
            "head_mode", 10, std::bind(&HeadMover::head_mode_callback, this, _1)); // should be callback group 1
    // ball_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    //  "ball_position_relative_filtered", 10, std::bind(&HeadMover::ball_filtered_callback, this, _1)); // Do I even need this? where do I use the ball?
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&HeadMover::joint_state_callback, this, _1)); // should be callback group 1

    // load parameters from config
    auto param_listener =
        std::make_shared<move_head::ParamListener>(rclcpp::Node::make_shared("head_mover")); //is this a problem?
    params_ = param_listener->get_params();
    pan_speed_ = params_.look_at.pan_speed;
    tilt_speed_ = params_.look_at.tilt_speed;

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/move_group");
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    rcl_interfaces::msg::ListParametersResult parameter_list =
        parameters_client->list_parameters({"robot_description_kinematics"}, 10);
    auto copied_parameters = parameters_client->get_parameters(parameter_list.names);

    // set the parameters to our node
    set_parameters(copied_parameters);

    std::string robot_description = "robot_description";
    // get the robot description from the blackboard
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
        robot_model_loader::RobotModelLoader(SharedPtr(this), robot_description, true));
    robot_model_ = loader_->getModel();
    if (!robot_model_) {
      RCLCPP_ERROR(this->get_logger(),
                   "failed to load robot model %s. Did you start the "
                   "blackboard (bitbots_utils base.launch)?",
                   robot_description.c_str());
    }
    robot_state_.reset(new moveit::core::RobotState(robot_model_));
    robot_state_->setToDefaultValues();

    // get planning scene for collision checking
    planning_scene_monitor_ =
        std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(SharedPtr(this), robot_description);
    planning_scene_ = planning_scene_monitor_->getPlanningScene();
    if (!planning_scene_) {
      RCLCPP_ERROR_ONCE(this->get_logger(), "failed to connect to planning scene");
    }


    // prepare the pos_msg
    pos_msg_.joint_names = {"HeadPan", "HeadTilt"};
    pos_msg_.positions = {0, 0};
    pos_msg_.velocities = {0, 0};
    pos_msg_.accelerations = {0, 0};
    pos_msg_.max_currents = {0, 0};


    // apparently tf_listener is necessary but unused
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // prepare joint state msg
    current_joint_state_ = sensor_msgs::msg::JointState();
    current_joint_state_.name = {"HeadPan", "HeadTilt"};
    current_joint_state_.position = {0, 0};
    current_joint_state_.velocity = {0, 0};
    current_joint_state_.effort = {0, 0};

    prev_head_mode = -1;
    threshold_ = params_.position_reached_threshold*DEG_TO_RAD;
    pan_speed = 0;
    tilt_speed = 0;

  }

  void head_mode_callback(const humanoid_league_msgs::msg::HeadMode::SharedPtr msg) {
    /**
     *ROS Subscriber callback for /head_mode message.
        Saves the messages head mode on the blackboard
     * 
     */
    head_mode_ = *msg;
  }
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    current_joint_state_ = *msg;
  }

/*
 HEAD POSITION
*/
  double calculate_lower_speed(double delta_fast_joint, double delta_my_joint, double speed) {
    double estimated_time = delta_fast_joint / speed;
    if (estimated_time != 0) {

      return delta_my_joint / estimated_time;
    } else {

      return 0;
    }
  };


  bool send_motor_goals(double pan_position,
                        double tilt_position,
                        bool resolve_collision,
                        double pan_speed = 1.5,
                        double tilt_speed = 1.5,
                        double current_pan_position = 0.0,
                        double current_tilt_position = 0.0,
                        bool clip = true) 
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "target pan/tilt: " << pan_position << "/" << tilt_position);
    if (clip) {
            std::pair<double, double> clipped = pre_clip(pan_position, tilt_position);
      pan_position = clipped.first;
      tilt_position = clipped.second;
    }

    if (resolve_collision) {
      bool success = avoid_collision_on_path(pan_position,
                                             tilt_position,
                                             current_pan_position,
                                             current_tilt_position,
                                             pan_speed,
                                             tilt_speed);
      if (!success) {

        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Unable to resolve head collision");
      }
      return success;
    } else {
      move_head_to_position_with_speed_adjustment(pan_position,
                                                  tilt_position,
                                                  current_pan_position,
                                                  current_tilt_position,
                                                  pan_speed,
                                                  tilt_speed);
      return true;
    }

  };

  bool send_motor_goals(double pan_position,
                        double tilt_position,
                        double pan_speed = 1.5,
                        double tilt_speed = 1.5,
                        bool clip = true) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "target pan/tilt: " << pan_position << "/" << tilt_position);
    if (clip) {
      std::pair<double, double> clipped = pre_clip(pan_position, tilt_position);
      pan_position = clipped.first;
      tilt_position = clipped.second;
    }

    pos_msg_.positions = {pan_position, tilt_position};
    pos_msg_.velocities = {pan_speed, tilt_speed};
    pos_msg_.header.stamp = this->get_clock()->now();
    // log the pos_msg_ and say that its in sent motor goals
    position_publisher_->publish(pos_msg_);
    return true;

  };
 
std::pair<double, double> pre_clip(double pan, double tilt)
{
  double new_pan = std::min(std::max(pan, params_.max_pan[0]), params_.max_pan[1]);
  double new_tilt = std::min(std::max(tilt, params_.max_tilt[0]), params_.max_tilt[1]);
  return std::make_pair(new_pan, new_tilt);
}
  bool avoid_collision_on_path(double goal_pan,
                               double goal_tilt,
                               double current_pan,
                               double current_tilt,
                               double pan_speed,
                               double tilt_speed,
                               int max_depth = 4,
                               int depth = 0) {
    if (depth > max_depth) {
      move_head_to_position_with_speed_adjustment(0.0, 0.0, current_pan, current_tilt, pan_speed, tilt_speed);
      return false;
    }
    double distance = sqrt(pow(goal_pan - current_pan, 2) - pow(goal_tilt - current_tilt, 2));

    int step_count = distance / 3 * DEG_TO_RAD;

    // calculate path
    std::vector<std::pair<double, double>> pan_and_tilt_steps;
    for (int i = 0; i < step_count; i++) {
      pan_and_tilt_steps[i].first = current_pan + (goal_pan - current_pan) / step_count * i;
      pan_and_tilt_steps[i].second = current_tilt + (goal_tilt - current_tilt) / step_count * i;
    }
    // checks if we have collisions on our path
    for (int i = 0; i < step_count; i++) {
      if (check_head_collision(pan_and_tilt_steps[i].first, pan_and_tilt_steps[i].second)) {
        return avoid_collision_on_path(goal_pan,
                                       goal_tilt + 10 * DEG_TO_RAD,
                                       current_pan,
                                       current_tilt,
                                       pan_speed,
                                       tilt_speed,
                                       max_depth,
                                       depth + 1);
      }
    }
    move_head_to_position_with_speed_adjustment(goal_pan, goal_tilt, current_pan, current_tilt, pan_speed, tilt_speed);
    return true;
  };

  bool check_head_collision(double pan, double tilt) {
    sensor_msgs::msg::JointState joint_state = sensor_msgs::msg::JointState();
    joint_state.name = {"HeadPan", "HeadTilt"};
    joint_state.position = {pan, tilt}; // looked at bitbots_move_it_bindings, but where import collision_detection?
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
    planning_scene_->checkCollision(req, res, *robot_state_, acm);
    return res.collision;
  }
  void move_head_to_position_with_speed_adjustment(double goal_pan,
                                                   double goal_tilt,
                                                   double current_pan,
                                                   double current_tilt,
                                                   double pan_speed,
                                                   double tilt_speed) {
    double delta_pan = std::abs(goal_pan - current_pan);
    double delta_tilt = std::abs(goal_tilt - current_tilt);
    if (delta_pan > 0) {
      tilt_speed = calculate_lower_speed(delta_pan, delta_tilt, pan_speed);
    } else {
      pan_speed = calculate_lower_speed(delta_tilt, delta_pan, tilt_speed);
    }
    pos_msg_.positions = {goal_pan, goal_tilt};
    pos_msg_.velocities = {pan_speed, tilt_speed};
    pos_msg_.header.stamp = rclcpp::Clock().now();
    // log the pos_msg_ and say that its in move head to position with speed adjustment
    position_publisher_->publish(pos_msg_);
  }

  /*
  GET HEAD POSITION
  */
  std::pair<double, double> get_head_position() {
    // loop over all joints and find the head pan and tilt
    double head_pan;
    double head_tilt;
    for(int i = 0; i < current_joint_state_.name.size(); i++) {
      if (current_joint_state_.name[i] == "HeadPan") {
        head_pan = current_joint_state_.position[i];
      } else if (current_joint_state_.name[i] == "HeadTilt") {
        head_tilt = current_joint_state_.position[i];
      }
    }
    return {head_pan, head_tilt};
  }

  /*
  PATTERN GENERATOR
  */
  double lineAngle(int line, int line_count, double min_angle, double max_angle) {
    double delta = std::abs(max_angle - min_angle);
    int steps = delta / (line_count - 1);
    double value = steps * line + min_angle;
    return value;
  }

  double calculateHorizonAngle(bool is_right, double angle_right, double angle_left) {
    if (is_right) {

      return angle_right;
    } else {

      return angle_left;
    }
  }
  std::vector<std::pair<double, double>> interpolatedSteps(int steps, double tilt, double min_pan, double max_pan) {
    if (steps == 0) {
      return {};
    }
    steps += 1;
    std::vector<std::pair<double, double>> output_points;
    double delta = std::abs(max_pan - min_pan);
    double step_size = delta / steps;
    for (int i = 1; i <= steps; i++) {
      double pan = min_pan + step_size * i;
      output_points.push_back({pan, tilt});
    }
    return output_points;
  }
  std::vector<std::pair<double, double>> generatePattern(int line_count,
                                                         double max_horizontal_angle_left,
                                                         double max_horizontal_angle_right,
                                                         double max_vertical_angle_up,
                                                         double max_vertical_angle_down,
                                                         int reduce_last_scanline = 1,
                                                         int interpolation_steps = 0) {
    std::vector<std::pair<double, double>> keyframes;
    bool down_direction = false;
    bool right_side = false;
    bool right_direction = true;
    int line = line_count - 1;
    int iterations = std::max(line_count*2-2, 2);
    for (int i = 0; i < iterations; i++) {
      std::pair<double, double> current_point =
          {calculateHorizonAngle(right_side, max_horizontal_angle_right, max_horizontal_angle_left),
           lineAngle(line, line_count, max_vertical_angle_down, max_vertical_angle_up)};
      RCLCPP_WARN(this->get_logger(), "current point: %f, %f", current_point.first, current_point.second);
      keyframes.push_back(current_point);

      if (right_side != right_direction) {
        std::vector<std::pair<double, double>> interpolated_points = interpolatedSteps(interpolation_steps,
                                                                                       current_point.second,
                                                                                       max_horizontal_angle_right,
                                                                                       max_horizontal_angle_left);
        if (right_direction) {
          std::reverse(interpolated_points.begin(), interpolated_points.end());
        }
        keyframes.insert(keyframes.end(), interpolated_points.begin(), interpolated_points.end());
        right_side = right_direction;

      } else {
        right_side = !right_direction;
        if (0 <= line && line <= line_count - 1) {
          down_direction = !down_direction;
        }
        if (down_direction) {
          line -= 1;
        } else {
          line += 1;
        }
      }
    }
    int keyframe_size = keyframes.size();
    for (int i = 0; i < keyframe_size; i++) {
      if (keyframes[i].second == max_vertical_angle_down) {
        keyframes[i] = {keyframes[i].first * reduce_last_scanline, max_vertical_angle_down};
      }
    }
    // log the keyframes appropriately
    RCLCPP_WARN(this->get_logger(), "keyframes: ");
    for (int i = 0; i < keyframes.size(); i++) {
      RCLCPP_WARN(this->get_logger(), "keyframe %d: %f, %f", i, keyframes[i].first, keyframes[i].second);
    }
    return keyframes;
  }

// LookAt
  std::pair<double, double> get_motor_goals_from_point(geometry_msgs::msg::Point point) {
    // use bio ik
    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.return_approximate_solution = true;
    ik_options.replace = true;

    bio_ik::LookAtGoal goal;
    goal.setTarget({point.x, point.y, point.z});
    goal.setWeight(1.0);
    goal.setAxis({1.0, 0.0, 0.0});
    goal.setLinkName("camera");
    ik_options.goals.emplace_back(new bio_ik::LookAtGoal(goal));

    auto joint_model_group = robot_model_->getJointModelGroup("Head");

    double timeout_seconds = 1.0;
    bool success = robot_state_->setFromIK(joint_model_group, EigenSTL::vector_Isometry3d(), std::vector<std::string>(),
                                           timeout_seconds, moveit::core::GroupStateValidityCallbackFn(), ik_options);
    robot_state_->update();

    bio_ik_msgs::msg::IKResponse response;
    moveit::core::robotStateToRobotStateMsg(*robot_state_, response.solution);
    response.solution_fitness = ik_options.solution_fitness;
    if (success) {
      std::pair<double, double>
          states = {response.solution.joint_state.position[0], response.solution.joint_state.position[1]};
      return states;
    } else {
      std::cout << "BioIK failed with error code: " << response.error_code.val << std::endl;
      std::pair<double, double> states = {0.0, 0.0};
      return states;
    }

  }

// covers look_<direction>, but a direction needs to be specified
  void look_at(geometry_msgs::msg::PointStamped point, double min_pan_delta = 0.0, double min_tilt_delta = 0.0) {
    try {
      geometry_msgs::msg::PointStamped
          new_point = tf_buffer_->transform(point, head_tf_frame_, tf2::durationFromSec(0.9));

      std::pair<double, double> pan_tilt = get_motor_goals_from_point(new_point.point);
      std::pair<double, double> current_pan_tilt = get_head_position();
      if (std::abs(pan_tilt.first - current_pan_tilt.first) > min_pan_delta
          || std::abs(pan_tilt.second - current_pan_tilt.second)
              > min_tilt_delta) // can we just put the min_tilt_delta as radiant into the conrfig?
      {
        send_motor_goals(pan_tilt.first, pan_tilt.second, true); // watch that it takes the correct one
        // tilt_speed=self.tilt_speed,
        // current_pan_position=current_head_pan,
        // current_tilt_position=current_head_tilt,
        // resolve_collision=True);
      }
    }
    catch (const std::exception &e) {
      std::cerr << e.what() << '\n';
    }
  }

  // ball_seen decision
  // bool ball_seen() {

  //   bool ball_seen;


  //   if (ball_last_seen != rclcpp::Time::Time(0,
	// 	RCL_SYSTEM_TIME ) && this->get_clock()->now() - ball_last_seen < params_.ball_lost_time) { // clock needs to be Clocktype.ROS_TIME?
  //     return true;
  //   }
  //   return false;

  // }

int get_near_pattern_position(std::vector<std::pair<double, double>> pattern, double pan, double tilt)
{
  std::pair<double, int> min_distance_point = {10000.0, -1};
  for (int i = 0; i < pattern.size(); i++) {
    double point_pan = pattern[i].first*DEG_TO_RAD;
    double point_tilt = pattern[i].second*DEG_TO_RAD;
    double distance = std::sqrt(std::pow(pattern[i].first - pan, 2) + std::pow(pattern[i].second - tilt, 2));
    if (distance < min_distance_point.first) {
      min_distance_point.first = distance;
      min_distance_point.second = i;
    }
  }
  return min_distance_point.second;

}

void perform_search_pattern()
{
  // log the pattern size
  index_ = index_ % pattern_.size(); 
  double head_pan = pattern_[index_].first*DEG_TO_RAD;
  double head_tilt = pattern_[index_].second*DEG_TO_RAD;
  double current_head_pan;
  double current_head_tilt;
  std::pair<double, double> head_position = get_head_position();
  current_head_pan = head_position.first;
  current_head_tilt = head_position.second;
  
  bool success = send_motor_goals(head_pan, head_tilt, true, pan_speed, tilt_speed, current_head_pan, current_head_tilt);

  if (success) {
    double distance_to_goal = std::sqrt(std::pow(head_pan - current_head_pan, 2) + std::pow(head_tilt - current_head_tilt, 2));
    // log distance to goal
    RCLCPP_INFO(this->get_logger(), "distance to goal: %f", distance_to_goal);
    // log threshold
    RCLCPP_INFO(this->get_logger(), "threshold: %f", threshold_);
    if (distance_to_goal <= threshold_) {index_++;
    // log that index was increased
    RCLCPP_INFO(this->get_logger(), "index was increased");}     
    }
  else{
// log that success was false
RCLCPP_INFO(this->get_logger(), "success was false");
    index_++;}

}

  void behave() {
  // log the current head mode
  //log the head_mode_.BALL_MODE
  uint curr_head_mode = head_mode_.head_mode;

  double current_head_pan;
  double current_head_tilt;

  //  The modes i want are: 1. ball search pattern, 2. penaltix search pattern, 3. field feature search paattern, 4. fron search pattern 

  if (prev_head_mode != curr_head_mode)
  {
    // log current head mode
    RCLCPP_INFO(this->get_logger(), "current head mode: %d", curr_head_mode);
  switch (curr_head_mode) {
    case head_mode_.BALL_MODE: // 0
      pan_speed = params_.search_pattern.pan_speed; // change this value depending on the head mode
      tilt_speed = params_.search_pattern.tilt_speed; // same as above
  pattern_ = generatePattern(params_.search_pattern.scan_lines, params_.search_pattern.pan_max[0], params_.search_pattern.pan_max[1], params_.search_pattern.tilt_max[0], params_.search_pattern.tilt_max[1]); // todo params
      break;
    case head_mode_.BALL_MODE_PENALTY: // 11
      pan_speed = params_.search_pattern_penalty.pan_speed; // change this value depending on the head mode
      tilt_speed = params_.search_pattern_penalty.tilt_speed; // same as above
      pattern_ = generatePattern(params_.search_pattern_penalty.scan_lines, params_.search_pattern_penalty.pan_max[0], params_.search_pattern_penalty.pan_max[1], params_.search_pattern_penalty.tilt_max[0], params_.search_pattern_penalty.tilt_max[1]); // todo params
    break;

    case head_mode_.FIELD_FEATURES: // 3
      pan_speed = params_.search_pattern_field_features.pan_speed;
      tilt_speed = params_.search_pattern_field_features.tilt_speed; 
      pattern_ = generatePattern(params_.search_pattern_field_features.scan_lines, params_.search_pattern_field_features.pan_max[0], params_.search_pattern_field_features.pan_max[1], params_.search_pattern_field_features.tilt_max[0], params_.search_pattern_field_features.tilt_max[1]); // todo params
    break;

    case head_mode_.LOOK_FRONT: // 13
      pan_speed = params_.front_search_pattern.pan_speed;
      tilt_speed = params_.front_search_pattern.tilt_speed; 
      pattern_ = generatePattern(params_.front_search_pattern.scan_lines, params_.front_search_pattern.pan_max[0], params_.front_search_pattern.pan_max[1], params_.front_search_pattern.tilt_max[0], params_.front_search_pattern.tilt_max[1]); // todo params
    break;
  std::pair<double, double> head_position = get_head_position();
  current_head_pan = head_position.first;
   current_head_tilt = head_position.second;

  index_ = get_near_pattern_position(pattern_, current_head_pan, current_head_tilt);
    // case head_mode_.FIELD_FEATURES:
    // case head_mode_.LOOK_FRONT:
  }
 // todo: params
  prev_head_mode = curr_head_mode;

  }
  perform_search_pattern();
  };
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadMover>());
  rclcpp::shutdown();

  return 0;
}
