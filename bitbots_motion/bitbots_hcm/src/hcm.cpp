#include <pybind11/embed.h>

#include <chrono>
#include <iostream>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_python_extension/serialization.hpp>
#include <thread>

#include "bitbots_msgs/msg/animation.hpp"
#include "bitbots_msgs/msg/foot_pressure.hpp"
#include "bitbots_msgs/msg/joint_command.hpp"
#include "bitbots_msgs/msg/robot_control_state.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;
namespace py = pybind11;
namespace bitbots_hcm {

class HCM_CPP : public rclcpp::Node {
 public:
  explicit HCM_CPP()
      : Node("hcm_cpp",
             rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(
                 true)) {
    // These are provided by the launch and not in the yaml file therefore we need to handle them seperatly
    bool use_sim_time, simulation_active, visualization_active;
    this->get_parameter("use_sim_time", use_sim_time);
    this->get_parameter("simulation_active", simulation_active);
    this->get_parameter("visualization_active", visualization_active);

    // Initialize HCM logic
    // Import Python module
    // "from bitbots_hcm.humanoid_control_module import HardwareControlManager"
    auto hcm_module = py::module::import("bitbots_hcm.humanoid_control_module");

    // Create HCM object
    // hcm = HardwareControlManager()
    hcm_py_ = hcm_module.attr("HardwareControlManager")(use_sim_time, simulation_active, visualization_active);

    // Create publishers
    pub_controller_command_ = this->create_publisher<bitbots_msgs::msg::JointCommand>("DynamixelController/command", 1);
    pub_robot_state_ = this->create_publisher<bitbots_msgs::msg::RobotControlState>("robot_state", 1);

    // Create subscribers for goals
    anim_sub_ = this->create_subscription<bitbots_msgs::msg::Animation>(
        "animation", 1, std::bind(&HCM_CPP::animation_callback, this, _1));
    dynup_sub_ = this->create_subscription<bitbots_msgs::msg::JointCommand>(
        "dynup_motor_goals", 1, std::bind(&HCM_CPP::dynup_callback, this, _1));
    head_sub_ = this->create_subscription<bitbots_msgs::msg::JointCommand>(
        "head_motor_goals", 1, std::bind(&HCM_CPP::head_goal_callback, this, _1));
    kick_sub_ = this->create_subscription<bitbots_msgs::msg::JointCommand>(
        "kick_motor_goals", 1, std::bind(&HCM_CPP::kick_goal_callback, this, _1));
    record_sub_ = this->create_subscription<bitbots_msgs::msg::JointCommand>(
        "record_motor_goals", 1, std::bind(&HCM_CPP::record_goal_callback, this, _1));
    walk_sub_ = this->create_subscription<bitbots_msgs::msg::JointCommand>(
        "walking_motor_goals", 1, std::bind(&HCM_CPP::walking_goal_callback, this, _1));

    // Create subscriber for high frequency sensor data
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, std::bind(&HCM_CPP::joint_state_callback, this, _1), high_hz_sub_options());
    pressure_l_sub_ = this->create_subscription<bitbots_msgs::msg::FootPressure>(
        "foot_pressure_left/filtered", 1, std::bind(&HCM_CPP::pressure_l_callback, this, _1), high_hz_sub_options());
    pressure_r_sub_ = this->create_subscription<bitbots_msgs::msg::FootPressure>(
        "foot_pressure_right/filtered", 1, std::bind(&HCM_CPP::pressure_r_callback, this, _1), high_hz_sub_options());
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 1, std::bind(&HCM_CPP::imu_callback, this, _1), high_hz_sub_options());
  }

  rclcpp::SubscriptionOptions high_hz_sub_options() {
    // Setup a MutuallyExclusive callback group for each high frequency
    // subscription so that they don't block each other
    rclcpp::SubscriptionOptions options;
    options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    return options;
  }

  void animation_callback(bitbots_msgs::msg::Animation msg) {
    // The animation server is sending us goal positions for the next keyframe
    last_animation_goal_time_ = msg.joint_command.header.stamp;

    // Forward joint positions to motors if there are any and we're in the right state
    // The right state is either one of the animation robot control states or if the animation is from the HCM
    if (msg.joint_command.positions.size() > 0 and
        ((current_state_ == bitbots_msgs::msg::RobotControlState::ANIMATION_RUNNING or
          current_state_ == bitbots_msgs::msg::RobotControlState::RECORD) or
         msg.from_hcm)) {
      // We can forward the animation goal to the motors
      pub_controller_command_->publish(msg.joint_command);
    }
  }

  void dynup_callback(const bitbots_msgs::msg::JointCommand msg) {
    if (msg.from_hcm and (current_state_ == bitbots_msgs::msg::RobotControlState::STARTUP ||
                          current_state_ == bitbots_msgs::msg::RobotControlState::GETTING_UP ||
                          current_state_ == bitbots_msgs::msg::RobotControlState::MOTOR_OFF ||
                          current_state_ == bitbots_msgs::msg::RobotControlState::PICKED_UP ||
                          current_state_ == bitbots_msgs::msg::RobotControlState::PENALTY ||
                          current_state_ == bitbots_msgs::msg::RobotControlState::CONTROLLABLE)) {
      pub_controller_command_->publish(msg);
    }
    if (current_state_ == bitbots_msgs::msg::RobotControlState::RECORD ||
        current_state_ == bitbots_msgs::msg::RobotControlState::CONTROLLABLE) {
      pub_controller_command_->publish(msg);
    }
  }

  void head_goal_callback(const bitbots_msgs::msg::JointCommand msg) {
    if (current_state_ == bitbots_msgs::msg::RobotControlState::CONTROLLABLE ||
        current_state_ == bitbots_msgs::msg::RobotControlState::WALKING) {
      pub_controller_command_->publish(msg);
    }
  }

  void kick_goal_callback(const bitbots_msgs::msg::JointCommand msg) {
    last_kick_time_ = msg.header.stamp;
    if (current_state_ == bitbots_msgs::msg::RobotControlState::KICKING ||
        current_state_ == bitbots_msgs::msg::RobotControlState::CONTROLLABLE) {
      // we can perform a kick
      pub_controller_command_->publish(msg);
    }
  }

  void record_goal_callback(const bitbots_msgs::msg::JointCommand msg) {
    if (msg.joint_names.size() == 0 && current_state_ == bitbots_msgs::msg::RobotControlState::RECORD) {
      pub_controller_command_->publish(msg);
    }
  }

  void walking_goal_callback(bitbots_msgs::msg::JointCommand msg) {
    last_walking_time_ = msg.header.stamp;
    if (current_state_ == bitbots_msgs::msg::RobotControlState::CONTROLLABLE ||
        current_state_ == bitbots_msgs::msg::RobotControlState::WALKING) {
      pub_controller_command_->publish(msg);
    }
  }

  void joint_state_callback(sensor_msgs::msg::JointState msg) { current_joint_state_ = msg; }

  void pressure_l_callback(bitbots_msgs::msg::FootPressure msg) { current_pressure_left_ = msg; }

  void pressure_r_callback(bitbots_msgs::msg::FootPressure msg) { current_pressure_right_ = msg; }

  void imu_callback(sensor_msgs::msg::Imu msg) { current_imu_ = msg; }

  void tick() {
    // Performs one tick of the HCM DSD

    // Pass the data we have got until now to the python module
    if (current_imu_) {
      hcm_py_.attr("set_imu")(ros2_python_extension::toPython(current_imu_.value()));
    }
    if (current_pressure_left_) {
      hcm_py_.attr("set_pressure_left")(
          ros2_python_extension::toPython<bitbots_msgs::msg::FootPressure>(current_pressure_left_.value()));
    }
    if (current_pressure_right_) {
      hcm_py_.attr("set_pressure_right")(
          ros2_python_extension::toPython<bitbots_msgs::msg::FootPressure>(current_pressure_right_.value()));
    }
    if (current_joint_state_)
      hcm_py_.attr("set_current_joint_state")(
          ros2_python_extension::toPython<sensor_msgs::msg::JointState>(current_joint_state_.value()));
    if (last_walking_time_) {
      hcm_py_.attr("set_last_walking_goal_time")(
          ros2_python_extension::toPython<builtin_interfaces::msg::Time>(last_walking_time_.value()));
    }
    if (last_kick_time_) {
      hcm_py_.attr("set_last_kick_goal_time")(
          ros2_python_extension::toPython<builtin_interfaces::msg::Time>(last_kick_time_.value()));
    }
    if (last_animation_goal_time_) {
      hcm_py_.attr("set_last_animation_goal_time")(
          ros2_python_extension::toPython<builtin_interfaces::msg::Time>(last_animation_goal_time_.value()));
    }

    // Run HCM Python DSD code
    hcm_py_.attr("tick")();

    // Pull the current robot state from the python module
    // It is used to perform the joint mutex
    py::object result = hcm_py_.attr("get_state")();
    current_state_ = result.cast<int>();

    // Publish current robot state
    bitbots_msgs::msg::RobotControlState state_msg = bitbots_msgs::msg::RobotControlState();
    state_msg.state = current_state_;
    pub_robot_state_->publish(state_msg);
  }

 private:
  // Python interpreter
  py::scoped_interpreter python_;
  // Python hcm module
  py::object hcm_py_;
  // The current robot state
  int current_state_ = bitbots_msgs::msg::RobotControlState::STARTUP;

  // Sensor states
  std::optional<sensor_msgs::msg::Imu> current_imu_;
  std::optional<bitbots_msgs::msg::FootPressure> current_pressure_left_;
  std::optional<bitbots_msgs::msg::FootPressure> current_pressure_right_;
  std::optional<sensor_msgs::msg::JointState> current_joint_state_;

  // Walking state
  std::optional<builtin_interfaces::msg::Time> last_walking_time_;

  // Kick state
  std::optional<builtin_interfaces::msg::Time> last_kick_time_;

  // Animation states
  std::optional<builtin_interfaces::msg::Time> last_animation_goal_time_;

  // Publishers
  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr pub_controller_command_;
  rclcpp::Publisher<bitbots_msgs::msg::RobotControlState>::SharedPtr pub_robot_state_;

  // Subscribers
  rclcpp::Subscription<bitbots_msgs::msg::Animation>::SharedPtr anim_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::JointCommand>::SharedPtr dynup_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::JointCommand>::SharedPtr head_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::JointCommand>::SharedPtr kick_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::JointCommand>::SharedPtr record_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::JointCommand>::SharedPtr walk_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr cop_l_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr cop_r_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_l_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_r_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};
}  // namespace bitbots_hcm

void thread_spin(rclcpp::experimental::executors::EventsExecutor::SharedPtr executor) { executor->spin(); }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bitbots_hcm::HCM_CPP>();

  rclcpp::experimental::executors::EventsExecutor::SharedPtr exec =
      std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
  exec->add_node(node);
  std::thread thread_obj(thread_spin, exec);

  auto last_time = node->get_clock()->now();
  rclcpp::Rate rate = rclcpp::Rate(125.0);
  while (rclcpp::ok()) {
    // Check if time progressed
    auto current_time = node->get_clock()->now();
    if (current_time > last_time) {
      last_time = current_time;
      node->tick();
      rate.sleep();
    }
    // really short sleep to not waste cpu time
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  // Join the thread
  thread_obj.join();

  rclcpp::shutdown();
}
