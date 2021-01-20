#include <bitbots_dynamic_kick/kick_pywrapper.h>

/**
 * Read a ROS message from a serialized string.
 */
template<typename M>
M from_python(const std::string &str_msg) {
  size_t serial_size = str_msg.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  for (size_t i = 0; i < serial_size; ++i) {
    buffer[i] = str_msg[i];
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  M msg;
  ros::serialization::Serializer<M>::read(stream, msg);
  return msg;
}

/**
 * Write a ROS message into a serialized string.
 */
template<typename M>
std::string to_python(const M &msg) {
  size_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, msg);
  std::string str_msg;
  str_msg.reserve(serial_size);
  for (size_t i = 0; i < serial_size; ++i) {
    str_msg.push_back(buffer[i]);
  }
  return str_msg;
}

PyKickWrapper::PyKickWrapper(const std::string ns) {
  kick_node_ = std::make_shared<bitbots_dynamic_kick::KickNode>(ns);
}

moveit::py_bindings_tools::ByteString PyKickWrapper::step(double dt) {
  std::string result = to_python<bitbots_msgs::JointCommand>(kick_node_->stepWrapper(dt));
  return moveit::py_bindings_tools::serializeMsg(result);
}

bool PyKickWrapper::init(const std::string &goal_str) {
  auto goal = from_python<bitbots_msgs::KickGoal>(goal_str);
  std::string error_string;
  return kick_node_->init(goal, error_string);
}

double PyKickWrapper::get_progress() {
  return kick_node_->getProgress();
}

BOOST_PYTHON_MODULE(py_dynamic_kick)
{
  using namespace boost::python;
  using namespace bitbots_dynamic_kick;

  class_<PyKickWrapper>("PyKickWrapper", init<std::string>())
      .def("init", &PyKickWrapper::init)
      .def("step", &PyKickWrapper::step)
      .def("get_progress", &PyKickWrapper::get_progress);
}
