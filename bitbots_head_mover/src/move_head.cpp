
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
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "rclcpp/clock.hpp"
#include <rclcpp/duration.hpp>


#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
//#include <tf_transformations/euler_from_quaternion.h>


#include <bio_ik/bio_ik.h>
#include <moveit/robot_state/conversions.h>
#include <tf2/convert.h>

#include <bio_ik_msgs/msg/ik_request.hpp>
#include <bio_ik_msgs/msg/ik_response.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/logger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/executors/events_executor/events_executor.hpp>

#include "rcl_interfaces/srv/get_parameters.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

class HeadMover : public rclcpp::Node
{

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

public:
  HeadMover()
  : Node("head_mover", rclcpp::NodeOptions().allow_undeclared_parameters(true))
  {

    RCLCPP_INFO(this->get_logger(), "Hello World 1");
    timer_ = this->create_wall_timer(
      500ms, std::bind(&HeadMover::behave, this));
RCLCPP_INFO(this->get_logger(), "Hello World 1");
    position_publisher_ = this->create_publisher<bitbots_msgs::msg::JointCommand>("head_motor_goals", 10); 
    head_mode_subscriber_ = this->create_subscription<humanoid_league_msgs::msg::HeadMode>( // here we want to call world_model.ball_filtered_callback
      "head_mode", 10, std::bind(&HeadMover::head_mode_callback, this, _1)); // should be callback group 1
    // ball_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    //  "ball_position_relative_filtered", 10, std::bind(&HeadMover::ball_filtered_callback, this, _1)); // Do I even need this? where do I use the ball?
      joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&HeadMover::joint_state_callback, this, _1)); // should be callback group 1

      // load parameters from config
       auto param_listener = std::make_shared<move_head::ParamListener>(rclcpp::Node::make_shared("head_mover")); //is this a problem?
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
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(SharedPtr(this), robot_description);
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

  // prepare world model msgs
  ball_ = geometry_msgs::msg::PointStamped();
  ball_odom_ = geometry_msgs::msg::PointStamped();
  ball_odom_.header.frame_id = odom_frame_; // todo: make this param later
  ball_odom_.header.stamp = this->now(); 
  ball_map_ = geometry_msgs::msg::PointStamped();
  ball_map_.header.frame_id = map_frame_; // todo: make this param later
  ball_map_.header.stamp = this->now();
  ball_teammate_ = geometry_msgs::msg::PointStamped();
  ball_teammate_.header.frame_id = map_frame_;
  ball_teammate_.header.stamp = this->now();
RCLCPP_INFO(this->get_logger(), "Hello World 1.5");


    RCLCPP_INFO(this->get_logger(), "Hello World 2");
  }
 static tf2::Vector3 p(const geometry_msgs::msg::Point& p) {
    return tf2::Vector3(p.x, p.y, p.z);
  }

  static tf2::Vector3 p(const geometry_msgs::msg::Vector3& p) {
    return tf2::Vector3(p.x, p.y, p.z);
  }

  static tf2::Quaternion q(const geometry_msgs::msg::Quaternion& q) {
    return tf2::Quaternion(q.x, q.y, q.z, q.w);
  }

  static double w(double w, double def = 1.0) {
    if (w == 0 || !std::isfinite(w))
      w = def;
    return w;
  }
 void head_mode_callback(const humanoid_league_msgs::msg::HeadMode::SharedPtr msg)
  {
    /**
     *ROS Subscriber callback for /head_mode message.
        Saves the messages head mode on the blackboard
     * 
     */
    head_mode_ = *msg; 
  }
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    current_joint_state_ = *msg;
    position_publisher_->publish(pos_msg_);
    
  }


/*
 HEAD POSITION
*/
  double calculate_lower_speed(double delta_fast_joint, double delta_my_joint, double speed)
  {
    double estimated_time = delta_fast_joint / speed;
    if (estimated_time != 0)
      {

        return delta_my_joint / estimated_time;
      }
    else
    {

      return 0;
    }
  };
 // if not given, resolve_collison is true
  bool send_motor_goals(double pan_position, double tilt_position, bool resolve_collision, double pan_speed = 1.5, double tilt_speed = 1.5, double current_pan_position = 0.0, double current_tilt_position = 0.0, bool clip = true) //TODO: make 2 methods
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "target pan/tilt: " << pan_position <<"/" << tilt_position);
    if (clip)
    {
      pan_position = std::min(std::max(pan_position, -1.5), 1.5); // TODO: use config instead of -1.5 and 1.5, that is what pre_clip does
      tilt_position = std::min(std::max(tilt_position, -0.5), 0.5);
    }

        if (resolve_collision)
        {
          bool success = avoid_collision_on_path(pan_position, tilt_position, current_pan_position, current_tilt_position, pan_speed, tilt_speed);
          if (!success){

                RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Unable to resolve head collision");
          }
          return success;
        }
        else
        {
          move_head_to_position_with_speed_adjustment(pan_position, tilt_position, current_pan_position, current_tilt_position, pan_speed, tilt_speed);
          return true;
        }

  };

    bool send_motor_goals(double pan_position, double tilt_position, double pan_speed = 1.5, double tilt_speed = 1.5, bool clip = true)
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "target pan/tilt: " << pan_position <<"/" << tilt_position);
    if (clip)
    {
      pan_position = std::min(std::max(pan_position, -1.5), 1.5); // TODO: use config instead of -1.5 and 1.5, that is what pre_clip does
      tilt_position = std::min(std::max(tilt_position, -0.5), 0.5);
    }


      pos_msg_.positions = {pan_position, tilt_position};
      pos_msg_.velocities = {pan_speed, tilt_speed};
      pos_msg_.header.stamp = this->get_clock()->now();
      position_publisher_->publish(pos_msg_);
      return true;
    
  };


  bool avoid_collision_on_path(double goal_pan, double goal_tilt, double current_pan, double current_tilt, double pan_speed, double tilt_speed, int max_depth = 4, int depth = 0)
  {
    if (depth > max_depth)
      {
        move_head_to_position_with_speed_adjustment(0.0, 0.0, current_pan, current_tilt, pan_speed, tilt_speed);
        return false;
      }
    double distance = sqrt(pow(goal_pan - current_pan, 2) - pow(goal_tilt - current_tilt, 2));

    int step_count = distance / 3 * DEG_TO_RAD;

    // calculate path
    std::vector<std::pair<double, double>> pan_and_tilt_steps;
    for (int i = 0; i < step_count; i++)
    {
      pan_and_tilt_steps[i].first = current_pan + (goal_pan - current_pan) / step_count * i;
      pan_and_tilt_steps[i].second = current_tilt + (goal_tilt - current_tilt) / step_count * i;
    }
    // checks if we have collisions on our path
    for (int i = 0; i < step_count; i++)
    {
      if (check_head_collision(pan_and_tilt_steps[i].first, pan_and_tilt_steps[i].second))
      {
        return avoid_collision_on_path(goal_pan, goal_tilt + 10 * DEG_TO_RAD, current_pan, current_tilt, pan_speed, tilt_speed, max_depth, depth + 1);
      }
    }
    move_head_to_position_with_speed_adjustment(goal_pan, goal_tilt, current_pan, current_tilt, pan_speed, tilt_speed);
    return true;
  };

  bool check_head_collision(double pan, double tilt)
  {
    sensor_msgs::msg::JointState joint_state = sensor_msgs::msg::JointState();
    joint_state.name = {"HeadPan", "HeadTilt"};
    joint_state.position = {pan, tilt}; // looked at bitbots_move_it_bindings, but where import collision_detection?
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
    planning_scene_->checkCollision(req, res, *robot_state_, acm);
    return res.collision;
  }
  void move_head_to_position_with_speed_adjustment(float goal_pan, float goal_tilt, float current_pan, float current_tilt, float pan_speed, float tilt_speed)
  {
    double delta_pan = std::abs(goal_pan - current_pan);
    double delta_tilt = std::abs(goal_tilt - current_tilt);
    if (delta_pan > 0)
      {
        tilt_speed = calculate_lower_speed(delta_pan, delta_tilt, pan_speed);
      }
    else
    {
      pan_speed = calculate_lower_speed(delta_tilt, delta_pan, tilt_speed);
    }
    pos_msg_.positions = {goal_pan, goal_tilt};
    pos_msg_.velocities = {pan_speed, tilt_speed};
    pos_msg_.header.stamp = rclcpp::Clock().now();
    position_publisher_->publish(pos_msg_);
  }

  /*
  GET HEAD POSITION
  */
  std::pair<double, double> get_head_position()
  {
    double head_pan = current_joint_state_.position[0];  // is this "HeadPan"?
    double head_tilt = current_joint_state_.position[1]; // is this "HeadTilt"?
    return {head_pan, head_tilt}; 
  }

  /*
  PATTERN GENERATOR
  */
  double lineAngle(int line, int line_count, double min_angle, double max_angle)
  {
    double delta = std::abs(max_angle - min_angle);
    int steps = delta / (line_count - 1);
    double value = steps * line + min_angle;
    return value;
  }

  double calculateHorizonAngle(bool is_right, bool angle_right, bool angle_left)
  {
    if (is_right)
    {

      return angle_right;
    }
    else
    {

      return angle_left;
    }
  }
  std::vector<std::pair<double, double>> interpolatedSteps(int steps, double tilt, double min_pan, double max_pan)
  {
    if(steps==0)
    {
      return {};
    }
    steps += 1;
    std::vector<std::pair<double, double>> output_points;
    double delta = std::abs(max_pan - min_pan);
    double step_size = delta / (steps);
    for (int i = 0; i < steps; i++)
    {
      double pan = min_pan + step_size * i;
      output_points.push_back({pan, tilt});
    }
    return output_points;
  }
  std::vector<std::pair<double, double>> generatePattern(int line_count, double max_horizontal_angle_left, double max_horizontal_angle_right, double max_vertical_angle_up, double max_vertical_angle_down, int reduce_last_scanline=1, int interpolation_steps=0)
  {
    std::vector<std::pair<double, double>> keyframes;
    bool down_direction = false;
    bool right_side = false;
    bool right_direction = true;
    int line = line_count -1;
    for (int i = 0; i < line_count*2 -2; i++)
    {
      std::pair<double, double> current_point = {calculateHorizonAngle(right_side, max_horizontal_angle_right, max_horizontal_angle_left), lineAngle(line, line_count, max_vertical_angle_down, max_vertical_angle_up)};
      keyframes.push_back(current_point);

      if (right_side != right_direction)
      {
        std::vector<std::pair<double, double>> interpolated_points = interpolatedSteps(interpolation_steps, current_point.second, max_horizontal_angle_right, max_horizontal_angle_left);
        if (right_direction)
        {
          std::reverse(interpolated_points.begin(), interpolated_points.end());
        }
        keyframes.insert(keyframes.end(), interpolated_points.begin(), interpolated_points.end());
        right_side = right_direction;
        
      }
      else
      {
        right_side = !right_direction;
        if (0 <= line && line <= line_count - 1)
        {
          down_direction = !down_direction;
        }
        if (down_direction)
        {
          line -= 1;
        }
        else
        {
          line += 1;
        }
      }
    }
    int keyframe_size = keyframes.size();
    for (int i = 0; i < keyframe_size; i++)
    {
      if (keyframes[i].second == max_vertical_angle_down)
      {
        keyframes[i] = {keyframes[i].first*reduce_last_scanline, max_vertical_angle_down};
      }
    }
    return keyframes;
  }

// LookAt
std::pair<double, double> get_motor_goals_from_point(geometry_msgs::msg::Point point)
{
  geometry_msgs::msg::Point target;
  target.x = point.x;
  target.y = point.y;
  target.z = point.z;
  // use bio ik
  bio_ik_msgs::msg::IKRequest::SharedPtr request = std::make_shared<bio_ik_msgs::msg::IKRequest>();
  request->group_name = "Head";
  request->timeout.sec = 1;
  request->approximate = true;
  // append look_at_goals
  bio_ik_msgs::msg::LookAtGoal goal;
  goal.target = target;
  goal.weight = 1.0;
  goal.axis.x = 1.0;
  goal.link_name = "camera";

  request->look_at_goals.push_back(goal);

  bio_ik_msgs::msg::IKResponse response = getBioIKIK(request);
  if (response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    std::pair<double, double> states = {response.solution.joint_state.position[0], response.solution.joint_state.position[1]};
    return states;
  }
else
{
  std::cout << "BioIK failed with error code: " << response.error_code.val << std::endl;
    std::pair<double, double> states = {0.0, 0.0};
  return states;
}

}

bio_ik_msgs::msg::IKResponse getBioIKIK(bio_ik_msgs::msg::IKRequest::SharedPtr msg) {
    // extra method to use BioIK specific goals
    bio_ik_msgs::msg::IKRequest request = *msg;

    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.return_approximate_solution = request.approximate;
    ik_options.fixed_joints = request.fixed_joints;
    ik_options.replace = true;

    convertGoals(request, ik_options);

    moveit::core::GroupStateValidityCallbackFn callback;
    if (request.avoid_collisions) {
      std::cout << "Avoid collisions not implemented in bitbots_moveit_bindings";
      exit(1);
    }
    auto joint_model_group = robot_model_->getJointModelGroup(request.group_name);
    //print joint model group
    std::cout << "Joint model group: " << request.group_name << std::endl;
    if (!joint_model_group) {
      std::cout << "Group name in IK call not specified";
      exit(1);
    }

    float timeout_seconds = request.timeout.sec + request.timeout.nanosec * 1e9;
    bool success = robot_state_->setFromIK(joint_model_group, EigenSTL::vector_Isometry3d(), std::vector<std::string>(),
                                           timeout_seconds, callback, ik_options);

    robot_state_->update();

    bio_ik_msgs::msg::IKResponse response;
    moveit::core::robotStateToRobotStateMsg(*robot_state_, response.solution);
    response.solution_fitness = ik_options.solution_fitness;
    if (success) {
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    } else {
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    }
    return response;
  }

    static void convertGoals(const bio_ik_msgs::msg::IKRequest& ik_request,
                           bio_ik::BioIKKinematicsQueryOptions& ik_options) {
    for (auto& m : ik_request.position_goals) {
      ik_options.goals.emplace_back(new bio_ik::PositionGoal(m.link_name, p(m.position), w(m.weight)));
    }

    for (auto& m : ik_request.orientation_goals) {
      ik_options.goals.emplace_back(new bio_ik::OrientationGoal(m.link_name, q(m.orientation), w(m.weight)));
    }

    for (auto& m : ik_request.pose_goals) {
      auto* g = new bio_ik::PoseGoal(m.link_name, p(m.pose.position), q(m.pose.orientation), w(m.weight));
      g->setRotationScale(w(m.rotation_scale, 0.5));
      ik_options.goals.emplace_back(g);
    }

    for (auto& m : ik_request.look_at_goals) {
      ik_options.goals.emplace_back(new bio_ik::LookAtGoal(m.link_name, p(m.axis), p(m.target), w(m.weight)));
    }

    for (auto& m : ik_request.min_distance_goals) {
      ik_options.goals.emplace_back(new bio_ik::MinDistanceGoal(m.link_name, p(m.target), m.distance, w(m.weight)));
    }

    for (auto& m : ik_request.max_distance_goals) {
      ik_options.goals.emplace_back(new bio_ik::MaxDistanceGoal(m.link_name, p(m.target), m.distance, w(m.weight)));
    }

    for (auto& m : ik_request.line_goals) {
      ik_options.goals.emplace_back(new bio_ik::LineGoal(m.link_name, p(m.position), p(m.direction), w(m.weight)));
    }

    for (auto& m : ik_request.avoid_joint_limits_goals) {
      ik_options.goals.emplace_back(new bio_ik::AvoidJointLimitsGoal(w(m.weight), !m.primary));
    }

    for (auto& m : ik_request.minimal_displacement_goals) {
      ik_options.goals.emplace_back(new bio_ik::MinimalDisplacementGoal(w(m.weight), !m.primary));
    }

    for (auto& m : ik_request.center_joints_goals) {
      ik_options.goals.emplace_back(new bio_ik::CenterJointsGoal(w(m.weight), !m.primary));
    }

    for (auto& m : ik_request.joint_variable_goals) {
      ik_options.goals.emplace_back(
          new bio_ik::JointVariableGoal(m.variable_name, m.variable_position, w(m.weight), m.secondary));
    }

    for (auto& m : ik_request.balance_goals) {
      auto* g = new bio_ik::BalanceGoal(p(m.target), w(m.weight));
      if (m.axis.x || m.axis.y || m.axis.z) {
        g->setAxis(p(m.axis));
      }
      ik_options.goals.emplace_back(g);
    }

    for (auto& m : ik_request.side_goals) {
      ik_options.goals.emplace_back(new bio_ik::SideGoal(m.link_name, p(m.axis), p(m.direction), w(m.weight)));
    }

    for (auto& m : ik_request.direction_goals) {
      ik_options.goals.emplace_back(new bio_ik::DirectionGoal(m.link_name, p(m.axis), p(m.direction), w(m.weight)));
    }

    for (auto& m : ik_request.cone_goals) {
      ik_options.goals.emplace_back(new bio_ik::ConeGoal(m.link_name, p(m.position), w(m.position_weight), p(m.axis),
                                                         p(m.direction), m.angle, w(m.weight)));
    }
  }

  void look_at(geometry_msgs::msg::PointStamped point, double min_pan_delta=0.0, double min_tilt_delta=0.0)
  {
    try
    {
      geometry_msgs::msg::PointStamped new_point = tf_buffer_->transform(point, head_tf_frame_, tf2::durationFromSec(0.9));

    std::pair<double, double> pan_tilt = get_motor_goals_from_point(new_point.point);
    std::pair<double, double> current_pan_tilt = get_head_position();
    if(std::abs(pan_tilt.first - current_pan_tilt.first) > min_pan_delta || std::abs(pan_tilt.second - current_pan_tilt.second) > min_tilt_delta) // can we just put the min_tilt_delta as radiant into the conrfig?
    {
      send_motor_goals(pan_tilt.first, pan_tilt.second, true); // watch that it takes the correct one
                                                          // tilt_speed=self.tilt_speed,
                                                          // current_pan_position=current_head_pan,
                                                          // current_tilt_position=current_head_tilt,
                                                          // resolve_collision=True);
    }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }

  void behave()
  {
    geometry_msgs::msg::PointStamped point;
    // print hello world
    RCLCPP_INFO(this->get_logger(), "Hello World");
    point.header.frame_id = "base_link";
    point.point.x = 1.0;
    point.point.y = 0.0;
    point.point.z = 0.0;

    look_at(point);
  }
;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadMover>());
  rclcpp::shutdown();
  
  return 0;
}
