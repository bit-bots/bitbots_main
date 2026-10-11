#include <memory>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "head_mover.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::experimental::executors::EventsExecutor exec;
  auto head_mover = std::make_shared<move_head::HeadMover>();
  exec.add_node(head_mover->get_node());

  // The detection callbacks block on tf lookups. Spinning them on a second
  // executor on a dedicated thread keeps that wait off the main executor, so the
  // search pattern timer keeps ticking at a steady rate. The detection callback
  // group was created with auto_add=false, so add_node above did not pick it up.
  rclcpp::executors::SingleThreadedExecutor detection_exec;
  detection_exec.add_callback_group(head_mover->get_detection_callback_group(),
                                    head_mover->get_node()->get_node_base_interface());
  std::thread detection_thread([&detection_exec]() { detection_exec.spin(); });

  exec.spin();

  detection_exec.cancel();
  detection_thread.join();
  rclcpp::shutdown();

  return 0;
}
