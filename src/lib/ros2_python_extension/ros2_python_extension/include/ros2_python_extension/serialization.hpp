#ifndef ROS2_PYTHON_EXTENSION_SERIALIZATION_HPP
#define ROS2_PYTHON_EXTENSION_SERIALIZATION_HPP

#include <pybind11/pybind11.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/duration.hpp>


namespace ros2_python_extension {
  namespace py = pybind11;

  /**
   * Convert the result of a ROS message serialized in Python to a C++ message
   * @tparam T C++ ROS message type
   * @param bytes message serialized in Python
   * @return the converted message
   */
  template<typename T>
  T fromPython(py::bytes &bytes) {
    // buffer_info is used to extract data pointer and size of bytes
    py::buffer_info info(py::buffer(bytes).request());

    // create the serialized message from python bytes object
    rclcpp::SerializedMessage serialized_message(info.size);
    serialized_message.get_rcl_serialized_message().buffer_length = info.size;
    memcpy(serialized_message.get_rcl_serialized_message().buffer, info.ptr, info.size);

    // do the deserialization
    T out;
    rclcpp::Serialization<T> serializer;
    serializer.deserialize_message(&serialized_message, &out);

    return out;
  }

  /**
   * Convert a C++ ROS message to a Python ByteString that can be deserialized to the message
   * @tparam T C++ ROS message type
   * @param msg C++ ROS message
   * @return the message serialized into a Python ByteString
   */
  template<typename T>
  py::bytes toPython(T &msg) {
    // serialize the message
    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_message;
    serializer.serialize_message(&msg, &serialized_message);

    // convert the serialized message to a python bytes object
    py::bytes out = {
      reinterpret_cast<const char *>(serialized_message.get_rcl_serialized_message().buffer),
      serialized_message.get_rcl_serialized_message().buffer_length
    };

    return out;
  }
} // namespace ros2_python_extension
#endif // ROS2_PYTHON_EXTENSION_SERIALIZATION_HPP
