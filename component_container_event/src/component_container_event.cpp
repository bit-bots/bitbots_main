#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

int main(int argc, char* argv[]) {
  /// Component container with an event-executor.
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
  exec->add_node(node);
  exec->spin();
}