#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_broadcaster.h>

#include <humanoid_league_msgs/msg/head_mode.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
//#include <bitbots_moveit_bindings/check_collision.hpp> use moveit directly

class HeadCapsule : public rclcpp::Node
{
public:
  HeadCapsule()
  : Node("head_capsule") // maybe use a node given as parameter and not inherit from node
  {
  }
  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr publisher_; // here I need to check what message type si sent
  //rclcpp::Publisher<visual_compass stuff>;
 std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void head_mode_callback(const humanoid_league_msgs::msg::HeadMode::SharedPtr msg)
  {
    head_mode_ = msg.head_mode;
  }
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    current_joint_state = msg;
  }
  
private:
  humanoid_league_msgs::msg::HeadMode head_mode_;

  bitbots_msgs::msg::JointCommand pos_msg;
  sensor_msgs::msg::JointState current_joint_state;

  float calculate_lower_speed(float delta_fast_joint, float delta_my_joint, float speed):
  {
    float estimated_time = delta_fast_joint / speed;
    if estimated_time != 0
    {

        return delta_my_joint / estimated_time;
    }
    else
    {

        return 0;
    }
  };

  bool send_motor_goals(auto pan_position, auto tilt_position, float pan_speed=1.5, float tilt_speed=1.5, auto current_pan_position = NULL, auto current_tilt_position= NULL, bool clip = true, bool resolve_collision = true)
  {

    get_logger()->debug("target pan/tilt: " + std::to_string(pan_position) + "/" + std::to_string(tilt_position));
    if clip
    {
        pan_position = std::min(std::max(pan_position, -1.5), 1.5); // TODO: use config instead of -1.5 and 1.5, that is what pre_clip does
        tilt_position = std::min(std::max(tilt_position, -0.5), 0.5);
    }
    
    if current_pan_position != NULL and current_tilt_position != NULL
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
            move_head_to_position_with_speed_adjustment(pan_position, tilt_position, current_pan_position,current_tilt_position, pan_speed, tilt_speed)
            return true;
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


    bool avoid_collision_on_path(auto goal_pan, auto goal_tilt, auto current_pan, auto current_tilt, auto pan_speed, auto tilt_speed, int max_depth=4, int depth=0)
    {
        if depth > max_depth
        {   move_head_to_position_with_speed_adjustment(0.0, 0.0, current_pan, current_tilt, pan_speed, tilt_speed);
            return false;
        }
        float distance = math::hypot(goal_pan - current_pan, goal_tilt - current_tilt);

        int step_count = distance / math.radians(3);

        // next in line is pan steps and tilt_steps and path and so on

        return true;
    };
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadCapsule>());
  rclcpp::shutdown();
  return 0;
}
