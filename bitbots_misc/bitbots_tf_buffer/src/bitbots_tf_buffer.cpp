#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <ros2_python_extension/serialization.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <utility>

namespace py = pybind11;

class Buffer {
 public:
  Buffer(py::bytes duration_raw, py::object node) {
    // initialize rclcpp if not already done
    if (!rclcpp::contexts::get_global_default_context()->is_valid()) {
      rclcpp::init(0, nullptr);
    }

    // get node name from python node object
    rcl_node_t *node_handle = (rcl_node_t *)node.attr("handle").attr("pointer").cast<size_t>();
    const char *node_name = rcl_node_get_name(node_handle);
    // create node with name <python_node_name>_tf
    node_ = std::make_shared<rclcpp::Node>((std::string(node_name) + "_tf").c_str());

    // Get buffer duration from python duration
    rclcpp::Duration duration{ros2_python_extension::fromPython<builtin_interfaces::msg::Duration>(duration_raw)};

    // create subscribers
    buffer_ = std::make_shared<tf2_ros::Buffer>(this->node_->get_clock());
    buffer_->setUsingDedicatedThread(true);
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node_, false);

    // create executor and start thread spinning the executor
    executor_ = std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
    executor_->add_node(node_);
    thread_ = std::make_shared<std::thread>([this]() {
      while (rclcpp::ok()) {
        executor_->spin_once();
      }
    });
  }

  py::bytes lookup_transform(py::str target_frame, py::str source_frame, py::bytes time_raw, py::bytes timeout_raw) {
    // lookup transform
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Time time_msg{ros2_python_extension::fromPython<builtin_interfaces::msg::Time>(time_raw)};
    rclcpp::Duration timeout{ros2_python_extension::fromPython<builtin_interfaces::msg::Duration>(timeout_raw)};

    try {
      transform = buffer_->lookupTransform(target_frame.cast<std::string>(), source_frame.cast<std::string>(), time_msg,
                                           timeout);
    } catch (tf2::TransformException &ex) {
      // throw python exception
      throw py::value_error(ex.what());
    }

    // serialize transform
    return ros2_python_extension::toPython<geometry_msgs::msg::TransformStamped>(transform);
  }

  bool can_transform(py::str target_frame, py::str source_frame, py::bytes time_raw, py::bytes timeout_raw) {
    // check if transform can be looked up
    rclcpp::Time time_msg{ros2_python_extension::fromPython<builtin_interfaces::msg::Time>(time_raw)};
    rclcpp::Duration timeout{ros2_python_extension::fromPython<builtin_interfaces::msg::Duration>(timeout_raw)};
    return buffer_->canTransform(target_frame.cast<std::string>(), source_frame.cast<std::string>(), time_msg, timeout);
  }

  // destructor
  ~Buffer() {
    // the executor finishes when rclcpp is shutdown, so the thread can be joined
    rclcpp::shutdown();
    thread_->join();
  }

 private:
  rclcpp::Serialization<builtin_interfaces::msg::Time> time_serializer_;
  rclcpp::Serialization<builtin_interfaces::msg::Duration> duration_serializer_;
  rclcpp::Serialization<geometry_msgs::msg::TransformStamped> transform_serializer_;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<std::thread> thread_;
  std::shared_ptr<rclcpp::experimental::executors::EventsExecutor> executor_;
};

PYBIND11_MODULE(cpp_wrapper, m) {
  py::class_<Buffer, std::shared_ptr<Buffer>>(m, "Buffer")
      .def(py::init<py::bytes, py::object>())
      .def("lookup_transform", &Buffer::lookup_transform)
      .def("can_transform", &Buffer::can_transform);
}
