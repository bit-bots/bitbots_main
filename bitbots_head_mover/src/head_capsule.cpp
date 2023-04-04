#include <cmath>
#include <rclcpp/rclcpp.hpp>
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
class HeadCapsule : public rclcpp::Node
{
public:

  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr position_publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  uint8_t head_mode_;
  sensor_msgs::msg::JointState current_joint_state_;
  robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningScenePtr planning_scene_;

  bitbots_msgs::msg::JointCommand pos_msg_;
  double DEG_TO_RAD = M_PI / 180;
  HeadCapsule()
      : Node("head_capsule")
  {


     std::string robot_description = "robot_description";
    // get the robot description from the blackboard
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      robot_model_loader::RobotModelLoader(SharedPtr(this), robot_description, true));
    robot_model_ = loader_->getModel();
    if (!robot_model_) {
      RCLCPP_ERROR(this->get_logger(),
                   "failed to load robot model %s. Did you start the "
                   "blackboard (bitbots_bringup base.launch)?",
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
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(SharedPtr(this), robot_description); 
  planning_scene_ = planning_scene_monitor_->getPlanningScene();

  // prepare the pos_msg
  pos_msg_.joint_names = {"HeadPan", "HeadTilt"};
  pos_msg_.positions = {0, 0};
  pos_msg_.velocities = {0, 0};
  pos_msg_.accelerations = {0, 0};
  pos_msg_.max_currents = {0, 0};

  // apparently tf_listener is necessary but unused
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // prepare joint state msg
  current_joint_state_ = sensor_msgs::msg::JointState();
  current_joint_state_.name = {"HeadPan", "HeadTilt"};
  current_joint_state_.position = {0, 0};
  current_joint_state_.velocity = {0, 0};
  current_joint_state_.effort = {0, 0};
  }

  void head_mode_callback(const humanoid_league_msgs::msg::HeadMode::SharedPtr msg)
  {
    /**
     *ROS Subscriber callback for /head_mode message.
        Saves the messages head mode on the blackboard
     * 
     */
    head_mode_ = msg->head_mode; 
  }
  void joint_state_callback(const sensor_msgs::msg::JointState msg)
  {
    current_joint_state_ = msg;
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

  bool send_motor_goals(double pan_position, double tilt_position, double pan_speed = 1.5, double tilt_speed = 1.5, double current_pan_position = 0.0, double current_tilt_position = 0.0, bool clip = true, bool resolve_collision = true) //TODO: make 2 methods
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
    for (int i = 0; i < keyframes.size(); i++)
    {
      if (keyframes[i].second == max_vertical_angle_down)
      {
        keyframes[i] = {keyframes[i].first*reduce_last_scanline, max_vertical_angle_down};
      }
    }
    return keyframes;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadCapsule>());
  rclcpp::shutdown();
  return 0;
}
