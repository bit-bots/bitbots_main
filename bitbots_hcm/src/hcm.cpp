// create hcm python with own spin thread
// create goal publisher
// subscribe to all goals
// subscribe to other high freq topics

// timer loop
    // update blackboard 
    // execute dsd
    // spin python ros

#include <pybind11/embed.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>

namespace py = pybind11;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hcm_cpp", rclcpp::NodeOptions()
                                                            .allow_undeclared_parameters(true)
                                                            .automatically_declare_parameters_from_overrides(true));
  // these are provided by the launch and not in the yaml file therefore we need to handle them seperatly
  bool use_sim_time, simulation_active, visualization_active;
  node->get_parameter("use_sim_time", use_sim_time);
  node->get_parameter("simulation_active", simulation_active);
  node->get_parameter("visualization_active", visualization_active);
  RCLCPP_WARN(node->get_logger(), "Hi hi!");

  std::cout << "hi\n";
  py::scoped_interpreter python;

  // from bitbots_hcm.humanoid_control_module import HardwareControlManager
  auto hcm_module = py::module::import("bitbots_hcm.humanoid_control_module");
  // hcm = HardwareControlManager()
  py::object hcm = hcm_module.attr("HardwareControlManager")(use_sim_time, simulation_active, visualization_active);

  while(true){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}