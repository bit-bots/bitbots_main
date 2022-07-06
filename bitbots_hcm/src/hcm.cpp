#include <pybind11/embed.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "bitbots_msgs/msg/foot_pressure.hpp"
#include "bitbots_msgs/msg/joint_command.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "humanoid_league_msgs/msg/animation.hpp"
#include "humanoid_league_msgs/msg/robot_control_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;
namespace py = pybind11;
namespace bitbots_hcm {

class HCM_CPP : public rclcpp::Node {
public:
  explicit HCM_CPP()
    : Node("hcm_cpp",
           rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(
               true)) {
    // these are provided by the launch and not in the yaml file therefore we need to handle them seperatly
    bool use_sim_time, simulation_active, visualization_active;
    this->get_parameter("use_sim_time", use_sim_time);
    this->get_parameter("simulation_active", simulation_active);
    this->get_parameter("visualization_active", visualization_active);

    current_state_ = humanoid_league_msgs::msg::RobotControlState::STARTUP;        

    // from bitbots_hcm.humanoid_control_module import HardwareControlManager
    auto hcm_module = py::module::import("bitbots_hcm.humanoid_control_module");
    // hcm = HardwareControlManager()
    hcm_py_ = hcm_module.attr("HardwareControlManager")(use_sim_time, simulation_active, visualization_active);

    // create goal publisher
    pub_controller_command_ = this->create_publisher<bitbots_msgs::msg::JointCommand>("walking_motor_goals", 1);

    // create subscriber motor goals
    anim_sub_ = this->create_subscription<humanoid_league_msgs::msg::Animation>(
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

    // subscriber for high frequency topics
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 1, std::bind(&HCM_CPP::joint_state_callback, this, _1));
    cop_l_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "cop_l", 1, std::bind(&HCM_CPP::cop_l_callback, this, _1));
    cop_r_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "cop_r", 1, std::bind(&HCM_CPP::cop_l_callback, this, _1));
    pressure_l_sub_ = this->create_subscription<bitbots_msgs::msg::FootPressure>(
        "foot_pressure_left/filtered", 1, std::bind(&HCM_CPP::pressure_l_callback, this, _1));
    pressure_r_sub_ = this->create_subscription<bitbots_msgs::msg::FootPressure>(
        "foot_pressure_right/filtered", 1, std::bind(&HCM_CPP::pressure_r_callback, this, _1));
    imu_sub_ =
        this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 1, std::bind(&HCM_CPP::imu_callback, this, _1));
  }

  void animation_callback(const humanoid_league_msgs::msg::Animation msg) {
    // The animation server is sending us goal positions for the next keyframe
    //  todo
    // self.blackboard.last_animation_goal_time = msg.header.stamp

    if (msg.request) {
      RCLCPP_INFO(this->get_logger(), "Got Animation request. HCM will try to get controllable now.");
      // animation has to wait
      // dsd should try to become controllable
      // todo
      // self.blackboard.animation_requested = True
      return;
    }
    if (msg.first) {
      if (msg.hcm) {
        // coming from ourselves
        // we don't have to do anything, since we must be in the right state
      } else {
        // coming from outside
        // check if we can run an animation now
        if (current_state_ != humanoid_league_msgs::msg::RobotControlState::CONTROLLABLE) {
          RCLCPP_WARN(this->get_logger(), "HCM is not controllable, animation refused.");
          // todo
          // self.blackboard.external_animation_running = True
        } else {
          // we're already controllable, go to animation running
          // todo
          // self.blackboard.external_animation_running = True
        }
      }
    }

    if (msg.last) {
      if (msg.hcm) {
        // This was an animation from the DSD
        // todo
        // self.blackboard.hcm_animation_finished = True
      } else {
        // this is the last frame, we want to tell the DSD that we're finished with the animations
        if (msg.position.points.size() == 0) {
          // probably this was just to tell us we're finished
          // we don't need to set another position to the motors
          return;
        }
      }
    }
    // forward positions to motors, if some where transmitted
    if (msg.position.points.size() > 0 && current_state_ != humanoid_league_msgs::msg::RobotControlState::GETTING_UP) {
      bitbots_msgs::msg::JointCommand out_msg = bitbots_msgs::msg::JointCommand();
      out_msg.positions = msg.position.points[0].positions;
      out_msg.joint_names = msg.position.joint_names;
      int number_joints = out_msg.joint_names.size();
      std::vector<double> values = {};
      for (int i = 0; i < number_joints; i++) {
        values.push_back(-1.0);
      }
      out_msg.accelerations = values;
      out_msg.velocities = values;
      out_msg.max_currents = values;
      if (msg.position.points[0].effort.size() != 0) {
        out_msg.max_currents = {};
        for (int i = 0; i < msg.position.points[0].effort.size(); i++) {
          out_msg.max_currents.push_back(msg.position.points[0].effort[i]);
        }
      }
      pub_controller_command_->publish(out_msg);
    }
  }

  void dynup_callback(const bitbots_msgs::msg::JointCommand msg) {
    if (current_state_ == humanoid_league_msgs::msg::RobotControlState::STARTUP ||
        current_state_ == humanoid_league_msgs::msg::RobotControlState::FALLEN ||
        current_state_ == humanoid_league_msgs::msg::RobotControlState::GETTING_UP ||
        current_state_ == humanoid_league_msgs::msg::RobotControlState::CONTROLLABLE) {
      pub_controller_command_->publish(msg);
    }
  }
  void head_goal_callback(const bitbots_msgs::msg::JointCommand msg) {
    if (current_state_ == humanoid_league_msgs::msg::RobotControlState::CONTROLLABLE ||
        current_state_ == humanoid_league_msgs::msg::RobotControlState::WALKING) {
      pub_controller_command_->publish(msg);
    }
  }
  void kick_goal_callback(const bitbots_msgs::msg::JointCommand msg) {
    if (current_state_ == humanoid_league_msgs::msg::RobotControlState::KICKING ||
        current_state_ == humanoid_league_msgs::msg::RobotControlState::CONTROLLABLE) {
      // we can perform a kick
      pub_controller_command_->publish(msg);
    }
  }
  void record_goal_callback(const bitbots_msgs::msg::JointCommand msg) {
    if (msg.joint_names.size() == 0) {
      // record tells us that its finished
      // todo
      // self.blackboard.record_active = False
    } else {
      // todo
      //  self.blackboard.record_active = True
      pub_controller_command_->publish(msg);
    }
  }
  void walking_goal_callback(const bitbots_msgs::msg::JointCommand msg) {
    // todo
    //  self.blackboard.last_walking_goal_time =self.node.get_clock().now()
    if (current_state_ == humanoid_league_msgs::msg::RobotControlState::CONTROLLABLE ||
        current_state_ == humanoid_league_msgs::msg::RobotControlState::WALKING) {
      pub_controller_command_->publish(msg);
    }
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // todo
    // self.blackboard.last_motor_update_time = msg.header.stamp
    // self.blackboard.previous_joint_state = self.blackboard.current_joint_state
    // self.blackboard.current_joint_state = msg
  }

  void cop_l_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // todo
    //          self.blackboard.cop_l_msg = msg
  }

  void cop_r_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // todo
    //          self.blackboard.cop_r_msg = msg
  }

  void pressure_l_callback(const bitbots_msgs::msg::FootPressure::SharedPtr msg) {
    // Gets new pressure values and writes them to the blackboard
    // todo
    // self.blackboard.last_pressure_update_time = msg.header.stamp
    // self.blackboard.pressures[0] = msg.left_front;
    // self.blackboard.pressures[1] = msg.left_back;
    // self.blackboard.pressures[2] = msg.right_front;
    // self.blackboard.pressures[3] = msg.right_back;
  }

  void pressure_r_callback(const bitbots_msgs::msg::FootPressure::SharedPtr msg) {
    // Gets new pressure values and writes them to the blackboard
    // todo
    // self.blackboard.last_pressure_update_time = msg.header.stamp
    // self.blackboard.pressures[4] = msg.left_front;
    // self.blackboard.pressures[5] = msg.left_back;
    // self.blackboard.pressures[6] = msg.right_front;
    // self.blackboard.pressures[7] = msg.right_back;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Gets new IMU values and computes the smoothed values of these
    /* todo
    self.blackboard.last_imu_update_time = msg.header.stamp

    self.blackboard.accel = numpy.array(
        [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    self.blackboard.gyro = numpy.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    self.blackboard.quaternion = numpy.array(
        ([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]))

    self.blackboard.smooth_gyro = numpy.multiply(self.blackboard.smooth_gyro, 0.95) + numpy.multiply(
        self.blackboard.gyro, 0.05)
    self.blackboard.smooth_accel = numpy.multiply(self.blackboard.smooth_accel, 0.99) + numpy.multiply(
        self.blackboard.accel, 0.01)
    self.blackboard.not_much_smoothed_gyro = numpy.multiply(self.blackboard.not_much_smoothed_gyro,
                                                            0.5) + numpy.multiply(self.blackboard.gyro, 0.5)

    self.blackboard.imu_msg = msg

    */
  }

  void loop() {
    // run HCM
    hcm_py_.attr("loop")();
    // update current HCM state for joint mutex
    py::object result = hcm_py_.attr("get_state")();
    current_state_ = result.cast<int>();    
  }

private:
  py::scoped_interpreter python_;
  py::object hcm_py_;
  int current_state_;

  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr pub_controller_command_;
  rclcpp::Subscription<humanoid_league_msgs::msg::Animation>::SharedPtr anim_sub_;
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

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bitbots_hcm::HCM_CPP>();

  rclcpp::Duration timer_duration = rclcpp::Duration::from_seconds(1.0 / 500);
  rclcpp::TimerBase::SharedPtr timer =
      rclcpp::create_timer(node, node->get_clock(), timer_duration, [node]() -> void { node->loop(); });
  rclcpp::executors::EventsExecutor exec;
  exec.add_node(node);

  exec.spin();
  rclcpp::shutdown();
}