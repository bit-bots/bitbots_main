#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/embed.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

#include <utility>

namespace py = pybind11;

class TransformListener {
public:
  TransformListener(py::object node, py::object py_wrapper) {
    // initialize rclcpp if not already done
    if (!rclcpp::contexts::get_global_default_context()->is_valid()) {
      rclcpp::init(0, nullptr);
    }

    // get node name from python node object
    rcl_node_t *node_handle = (rcl_node_t*) node.attr("handle").attr("pointer").cast<size_t>();
    const char *node_name = rcl_node_get_name(node_handle);
    // create node with name <python_node_name>_tf_listener
    node_ = std::make_shared<rclcpp::Node>((std::string(node_name) + "_tf_listener").c_str());

    // get python functions from wrapper object
    set_transform_ = py_wrapper.attr("set_transform");
    set_transform_static_ = py_wrapper.attr("set_transform_static");

    // create subscribers
    rclcpp::QoS qos(rclcpp::KeepLast(100));
    rclcpp::QoS qos_static(rclcpp::KeepLast(100));
    qos_static.transient_local();
    tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", qos, std::bind(&TransformListener::tf_callback, this, std::placeholders::_1));
    tf_static_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", qos_static, std::bind(&TransformListener::tf_static_callback, this, std::placeholders::_1));

    // create executor and start thread spinning the executor
    executor_ = std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
    executor_->add_node(node_);
    thread_ = std::make_shared<std::thread>([this]() {
      while (rclcpp::ok()) {
        executor_->spin_once();
      }
    });
  }

  // callbacks
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    common_callback(msg, set_transform_);
  }

  void tf_static_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    common_callback(msg, set_transform_static_);
  }

  // function with common code (serialization) from both callbacks
  void common_callback(const tf2_msgs::msg::TFMessage::SharedPtr &msg, py::object set_transform) {
    // the message is serialized to python to be able to transfer it to the python tf buffer
    auto serializer = rclcpp::Serialization<geometry_msgs::msg::TransformStamped>();
    rclcpp::SerializedMessage serialized_transform;

    for (auto &transform : msg->transforms) {
      // this is the actual serialization
      serializer.serialize_message(&transform, &serialized_transform);
      auto rcl_serialized_transform = serialized_transform.get_rcl_serialized_message();
      py::bytes py_serialized_transform = {
          reinterpret_cast<const char *>(rcl_serialized_transform.buffer),
          rcl_serialized_transform.buffer_length
      };
      // we have to acquire the GIL to be able to call python functions
      {
        // we need PyGILState_Ensure and py::gil_scoped_acquire because of a bug
        // see https://github.com/pybind/pybind11/issues/1920
        PyGILState_STATE gstate = PyGILState_Ensure();
        py::gil_scoped_acquire acquire;
        set_transform(py_serialized_transform);
      }
    }
  }

  // destructor
  ~TransformListener() {
    // the executor finishes when rclcpp is shutdown, so the thread can be joined
    rclcpp::shutdown();
    thread_->join();
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<std::thread> thread_;
  std::shared_ptr<rclcpp::experimental::executors::EventsExecutor> executor_;
  py::object set_transform_;
  py::object set_transform_static_;

  // subscriber objects
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
};

PYBIND11_MODULE(cpp_wrapper, m) {
  py::class_<TransformListener, std::shared_ptr<TransformListener>>(m, "TransformListener")
      .def(py::init<py::object, py::object>());
}
