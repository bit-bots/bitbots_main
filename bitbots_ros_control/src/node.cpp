#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <bitbots_ros_control/wolfgang_hardware_interface.h>
#include <signal.h>
#include <thread>
sig_atomic_t volatile request_shutdown = 0;

void sigintHandler(int sig) {
  // gives other nodes some time to perform shutdown procedures with robot
  request_shutdown = 1;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ros_control", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigintHandler);
  ros::NodeHandle pnh("~");

  // create hardware interfaces
  bitbots_ros_control::WolfgangHardwareInterface hw(pnh);

  if (!hw.init(pnh)) {
    ROS_ERROR_STREAM("Failed to initialize hardware interface.");
    return 1;
  }

  // Create separate queue, because otherwise controller manager will freeze
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(5);
  spinner.start();
  controller_manager::ControllerManager *cm = new controller_manager::ControllerManager(&hw, nh);
  // load controller directly here so that we have control when we shut down
  cm->loadController("joint_state_controller");
  cm->loadController("imu_sensor_controller");
  cm->loadController("DynamixelController");
  const std::vector<std::string> names = {"joint_state_controller", "imu_sensor_controller", "DynamixelController"};
  const std::vector<std::string> empty = {};

  // we have to start controller in own thread, otherwise it does not work, since the control manager needs to get its
  // first update before the controllers are started
  std::thread
      thread = std::thread(&controller_manager::ControllerManager::switchController, cm, names, empty, 2, true, 3);

  // Start control loop
  ros::Time current_time = ros::Time::now();
  ros::Duration period = ros::Time::now() - current_time;
  bool first_update = true;
  ros::Rate rate(pnh.param("control_loop_hz", 1000));
  ros::Time stop_time;
  bool shut_down_started = false;

  while (!request_shutdown || ros::Time::now().toSec() - stop_time.toSec() < 5) {
    hw.read(current_time, period);
    period = ros::Time::now() - current_time;
    current_time = ros::Time::now();

    // period only makes sense after the first update
    // therefore, the controller manager is only updated starting with the second iteration
    if (first_update) {
      first_update = false;
    } else {
      cm->update(current_time, period);
    }
    hw.write(current_time, period);
    ros::spinOnce();
    rate.sleep();

    if (request_shutdown && !shut_down_started) {
      stop_time = ros::Time::now();
      shut_down_started = true;
    }
  }
  thread.join();
  delete cm;
  ros::shutdown();
  return 0;
}
