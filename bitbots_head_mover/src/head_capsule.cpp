#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_broadcaster.h>

#include <humanoid_league_msgs/msg/head_mode.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include <bitbots_moveit_bindings/check_collision.hpp> use moveit directly

class HeadCapsule : public rclcpp::Node
{
public:
  HeadCapsule()
      : Node("head_capsule") // maybe use a node given as parameter and not inherit from node
  {
  }
  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr publisher_; // here I need to check what message type si sent
                                                                            // rclcpp::Publisher<visual_compass stuff>;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void head_mode_callback(const humanoid_league_msgs::msg::HeadMode::SharedPtr msg)
  {
    head_mode_ = msg->head_mode;
  }
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    current_joint_state = msg;
  }

private:
  int head_mode_;
  float DEG_TO_RAD = 3.141592653 / 180; // make this a macro later
  bitbots_msgs::msg::JointCommand pos_msg;
  sensor_msgs::msg::JointState::SharedPtr current_joint_state;

  float calculate_lower_speed(float delta_fast_joint, float delta_my_joint, float speed) :
  {
    float estimated_time = delta_fast_joint / speed;
    if estimated_time
      != 0
      {

        return delta_my_joint / estimated_time;
      }
    else
    {

      return 0;
    }
  };

  bool send_motor_goals(auto pan_position, auto tilt_position, float pan_speed = 1.5, float tilt_speed = 1.5, auto current_pan_position = NULL, auto current_tilt_position = NULL, bool clip = true, bool resolve_collision = true)
  {

    get_logger()->debug("target pan/tilt: " + std::to_string(pan_position) + "/" + std::to_string(tilt_position));
    if clip
    {
      pan_position = std::min(std::max(pan_position, -1.5), 1.5); // TODO: use config instead of -1.5 and 1.5, that is what pre_clip does
      tilt_position = std::min(std::max(tilt_position, -0.5), 0.5);
    }

    if current_pan_position
      != NULL and current_tilt_position != NULL
      {
        if resolve_collision
        {
          bool success = avoid_collision_on_path(pan_position, tilt_position, current_pan_position, current_tilt_position, pan_speed, tilt_speed);
          if !succes:
                get_logger()->error("Unable to resolve head collision");
          return success;
        }
        else
        {
          move_head_to_position_with_speed_adjustment(pan_position, tilt_position, current_pan_position, current_tilt_position, pan_speed, tilt_speed) return true;
        }
      }
    else
    {
      pos_msg.joint_names = {"HeadPan", "HeadTilt"};
      pos_msg.positions = {pan_position, tilt_position};
      pos_msg.speeds = {pan_speed, tilt_speed};
      pos_msg.accelerations = {0.0, 0.0};
      pos_msg.max_currents = {0.0, 0.0};
      pos_msg.modes = {0, 0};
      publisher_->publish(pos_msg);
      return true;
    }
  };

  bool avoid_collision_on_path(auto goal_pan, auto goal_tilt, auto current_pan, auto current_tilt, auto pan_speed, auto tilt_speed, int max_depth = 4, int depth = 0)
  {
    if depth
      > max_depth
      {
        move_head_to_position_with_speed_adjustment(0.0, 0.0, current_pan, current_tilt, pan_speed, tilt_speed);
        return false;
      }
    float distance = cmath::sqrt(cmath::pow(goal_pan - current_pan, 2) - cmath::pow(goal_tilt - current_tilt, 2));

    int step_count = distance / 3 * DEG_TO_RAD;

    // calculate path
    double pan_and_tilt_steps[step_count][2];
    for (int i = 0; i < step_count; i++)
    {
      pan_and_tilt_steps[i][0] = current_pan + (goal_pan - current_pan) / step_count * i;
      pan_and_tilt_steps[i][1] = current_tilt + (goal_tilt - current_tilt) / step_count * i;
    }
    // checks if we have collisions on our path
    if check_head_collision (goal_pan, goal_tilt)
    {
      return avoid_collision_on_path(goal_pan, goal_tilt + 10 * DEG_TO_RAD, current_pan, current_tilt, pan_speed, tilt_speed, max_depth, depth + 1);
    }
    for (int i = 0; i < step_count; i++)
    {
      if check_head_collision (pan_and_tilt_steps[i][0], pan_and_tilt_steps[i][1])
      {
        return avoid_collision_on_path(goal_pan, goal_tilt + 10 * DEG_TO_RAD, current_pan, current_tilt, pan_speed, tilt_speed, max_depth, depth + 1);
      }
    }
    move_head_to_position_with_speed_adjustment(goal_pan, goal_tilt, current_pan, current_tilt, pan_speed, tilt_speed);
    return true;
  };

  bool check_head_collision(auto head_joints)
  {
    sensor_msgs::msg::JointState joint_state = new sensor_msgs::msg::JointState();
    joint_state.name = {"HeadPan", "HeadTilt"};
    joint_state.position = head_joints; // looked at bitbots_move_it_bindings, but where import collision_detection?
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
    planning_scene_->checkCollision(req, res, *joint_state, acm);
    return res.collision;
  }
  void move_head_to_position_with_speed_adjustment(auto goal_pan, auto goal_tilt, auto current_pan, auto current_tilt, auto pan_speed, auto tilt_speed)
  {
    float delta_pan = std::abs(goal_pan - current_pan);
    float delta_tilt = std::abs(goal_tilt - current_tilt);
    if delta_pan
      > 0
      {
        tilt_speed = calculate_lower_speed(delta_pan, delta_tilt, pan_speed);
      }
    else
    {
      pan_speed = calculate_lower_speed(delta_tilt, delta_pan, tilt_speed);
    }
    pos_msg.positions = {goal_pan, goal_tilt};
    pos_msg.velocities = {pan_speed, tilt_speed};
    pos_msg.header.stamp = rclcpp::Clock().now().to_msg();
    publisher_->publish(pos_msg);
  }
  float get_head_position()
  {
    float head_pan = current_joint_state->position[0];  // is this "HeadPan"?
    float head_tilt = current_joint_state->position[1]; // is this "HeadTilt"?
    return head_pan, head_tilt;
  }
  float lineAngle(auto line, auto line_count, auto min_angle, auto max_angle)
  {
    float delta = std::abs(max_angle - min_angle);
    float steps = delta / (line_count - 1);
    float value = steps * line + min_angle;
    return value;
  }

  float calculateHorizonAngle(auto is_right, auto angle_right, auto angle_left) :
  {
    if is_right
    {

      return angle_right;
    }
    else
    {

      return angle_left;
    }
  }

  float interpolatedSteps(auto steps, float tilt, float min_pan, float max_pan)
  {
    if (stepps == 0)
    {
      return 0;
    }
    steps += 1.0;
    float delta = std::abs(max_pan - min_pan);
    float step_size = delta / steps;
    int[steps][2] output_points; // do we need to cast steps to int?
    for (int i = 0; i < steps; i++)
    {
      output_points[i][0] = min_pan + step_size * i; // to int aswell
      output_points[i][1] = tilt;
    }
    return output_points;
  }
  float generate_pattern(auto lineCount, auto maxHorizontalAngleLeft, auto maxHorizontalAngleRight, auto maxVerticalAngleUp, auto maxVerticalAngleDown, int reduce_last_scanline = 1, int interpolation_steps = 0)
  {
    auto keyframes; // how do I declare this?
    bool downDirection = false;
    bool rightSide = false;
    bool rightDirection = true;
    int line = lineCount - 1;
    int iterations = cmath.max(2 * lineCount - 2, 2);
    float[2] currentPoint;
    float[interpolation_steps] interpolatedKeyframes;
    for (int i = 0; i < iterations; i++)
    {
      currentPoint[0] = calculateHorizonAngle(rightSide, maxHorizontalAngleRight, maxHorizontalAngleLeft);
      currentPoint[1] = lineAngle(line, lineCount, maxVerticalAngleDown, maxVerticalAngleUp);
      keyframes[i] = currentPoint;

      if (rightSide != rightDirection)
      {
        interpolatedKeyframes = interpolatedSteps(interpolation_steps, currentPoint[1], currentPoint[0], currentPoint[0]);
        if (rightDirection)
        {
          interpolatedKeyframes.reverse(); // todo: reverse
        }
        for (int j = 0; j < interpolation_steps; j++)
        {
          keyframes[i + j] = interpolatedKeyframes[j];
        }
        i += interpolation_steps;
      }

      if (rightSide != rightDirection) // why does it say "; expected"? The function or condition seems not to be the problem
      {
        rightSide = rightDirection;
      }
      else
      {
        rightDirection = !rightDirection;
        if (line >= 0 and line <= lineCount - 1)
        {
          downDirection = !downDirection;
        }
        if downDirection
        {
          line -= 1;
        }
        else
        {
          line += 1;
        }
      }
    }

    int size = std::sizeof(keyframes);
    for (int i = 0; i < size; i++) // here declare is making a problem
    {
      auto keyframe = keyframes[i];
      if (keyframe[1] == maxVerticalAngleDown)
      {
        keyframes[i][0] = keyframe[0] * reduce_last_scanline;
        keyframes[i][1] = maxVerticalAngleDown;
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
