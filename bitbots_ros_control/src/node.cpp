#include <controller_manager/controller_manager.hpp>
#include <bitbots_ros_control/wolfgang_hardware_interface.h>
#include <signal.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/events_executor/events_executor.hpp>

sig_atomic_t volatile request_shutdown = 0;

void sigintHandler(int sig) {
  // gives other nodes some time to perform shutdown procedures with robot
  request_shutdown = 1;
}

int main(int argc, char *argv[]) {
  // register signal handler for ctrl-c that we use to request shutdown but give extra time for other nodes to finish
  signal(SIGINT, sigintHandler);

  // initialize ros
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).allow_undeclared_parameters(true);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("wolfgang_hardware_interface", options);

  // create hardware interfaces
  bitbots_ros_control::WolfgangHardwareInterface hw(nh);
  if (!hw.init()) {
    RCLCPP_ERROR(nh->get_logger(), "Failed to initialize hardware interface.");
    return 1;
  }

  // diagnostics
  int diag_counter = 0;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub = nh->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diagnostic_msgs::msg::DiagnosticArray array_msg = diagnostic_msgs::msg::DiagnosticArray();
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> array = std::vector<diagnostic_msgs::msg::DiagnosticStatus>();
  diagnostic_msgs::msg::DiagnosticStatus status = diagnostic_msgs::msg::DiagnosticStatus();
  // add prefix PS for pressure sensor to sort in diagnostic analyser
  status.name = "BUSBus";
  status.hardware_id = "Bus";

  // Start control loop
  rclcpp::Time current_time = nh->get_clock()->now();
  rclcpp::Duration period = nh->get_clock()->now() - current_time;
  bool first_update = true;
  float control_loop_hz = 500.0;
  nh->get_parameter("control_loop_hz", control_loop_hz);
  rclcpp::Rate rate(control_loop_hz);
  rclcpp::Time stop_time;
  bool shut_down_started = false;
  rclcpp::executors::EventsExecutor exec;
  exec.add_node(nh);


  while (!request_shutdown || nh->get_clock()->now().seconds() - stop_time.seconds() < 5) {
    //
    // read
    //
    hw.read(current_time, period);
    period = nh->get_clock()->now() - current_time;
    current_time = nh->get_clock()->now();

    // period only makes sense after the first update
    // therefore, the controller manager is only updated starting with the second iteration
    if (first_update) {
      first_update = false;
    } else {
      //todo replaced controller part, if necessary
    }

    //
    // Write
    //
    hw.write(current_time, period);
    exec.spin_some();
    rate.sleep();

    //
    // Diagnostics
    //
    // publish diagnostic messages each 100 frames
    if (diag_counter % 100 == 0) {
      // check if we are staying the correct cycle time. warning if we only get half
      array_msg.header.stamp = nh->get_clock()->now();
      if (rate.period() < std::chrono::nanoseconds (int(1e9 * (1 / control_loop_hz) * 2))) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "";
      } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Bus runs not at specified frequency";
      }
      array = std::vector<diagnostic_msgs::msg::DiagnosticStatus>();
      array.push_back(status);
      array_msg.status = array;
      diagnostic_pub->publish(array_msg);
    }
    diag_counter++;

    if (request_shutdown && !shut_down_started) {
      stop_time = nh->get_clock()->now();
      shut_down_started = true;
      RCLCPP_INFO(nh->get_logger(), "Shutting down in 5 seconds");
    }
  }
  return 0;
}
