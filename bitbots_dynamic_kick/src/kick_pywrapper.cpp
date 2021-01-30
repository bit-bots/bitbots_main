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

void PyKickWrapper::set_params(const boost::python::object params) {
  using namespace boost::python;

  extract<dict> cppdict_ext(params);
  if (!cppdict_ext.check()) {
    throw std::runtime_error("type error: not a python dict");
  }

  dict cppdict = cppdict_ext();
  list keylist = cppdict.keys();

  bitbots_dynamic_kick::DynamicKickConfig conf;

  int len = boost::python::len(keylist);

  for (int i = 0; i < len; ++i) {
    std::string keystr = extract<std::string>(str(keylist[i]));
    std::string valstr = extract<std::string>(str(cppdict[keylist[i]]));
    if (keystr == "engine_rate") { conf.engine_rate = std::stoi(valstr); }
    else if (keystr == "foot_rise") { conf.foot_rise = std::stof(valstr); }
    else if (keystr == "foot_distance") { conf.foot_distance = std::stof(valstr); }
    else if (keystr == "kick_windup_distance") { conf.kick_windup_distance = std::stof(valstr); }
    else if (keystr == "trunk_height") { conf.trunk_height = std::stof(valstr); }
    else if (keystr == "trunk_roll") { conf.trunk_roll = std::stof(valstr); }
    else if (keystr == "trunk_pitch") { conf.trunk_pitch = std::stof(valstr); }
    else if (keystr == "trunk_yaw") { conf.trunk_yaw = std::stof(valstr); }
    else if (keystr == "move_trunk_time") { conf.move_trunk_time = std::stof(valstr); }
    else if (keystr == "raise_foot_time") { conf.raise_foot_time = std::stof(valstr); }
    else if (keystr == "move_to_ball_time") { conf.move_to_ball_time = std::stof(valstr); }
    else if (keystr == "kick_time") { conf.kick_time = std::stof(valstr); }
    else if (keystr == "move_back_time") { conf.move_back_time = std::stof(valstr); }
    else if (keystr == "lower_foot_time") { conf.lower_foot_time = std::stof(valstr); }
    else if (keystr == "move_trunk_back_time") { conf.move_trunk_back_time = std::stof(valstr); }
    else if (keystr == "choose_foot_corridor_width") { conf.choose_foot_corridor_width = std::stof(valstr); }
    else if (keystr == "use_center_of_pressure") { conf.use_center_of_pressure = valstr == "True"; }
    else if (keystr == "stabilizing_point_x") { conf.stabilizing_point_x = std::stof(valstr); }
    else if (keystr == "stabilizing_point_y") { conf.stabilizing_point_y = std::stof(valstr); }
    else if (keystr == "spline_smoothness") { conf.spline_smoothness = std::stoi(valstr); }
    else { std::cerr << keystr << " not known. WILL BE IGNORED." << std::endl; }
  }

  kick_node_->reconfigureCallback(conf, 0xff);
}

bool PyKickWrapper::init(const std::string &goal_str) {
  auto goal = from_python<bitbots_msgs::KickGoal>(goal_str);
  std::string error_string;
  Eigen::Isometry3d trunk_to_base_footprint;
  return kick_node_->init(goal, error_string, trunk_to_base_footprint);
}

double PyKickWrapper::get_progress() {
  return kick_node_->getProgress();
}

BOOST_PYTHON_MODULE (py_dynamic_kick) {
  using namespace boost::python;
  using namespace bitbots_dynamic_kick;

  class_<PyKickWrapper>("PyKickWrapper", init<std::string>())
      .def("init", &PyKickWrapper::init)
      .def("step", &PyKickWrapper::step)
      .def("get_progress", &PyKickWrapper::get_progress)
      .def("set_params", &PyKickWrapper::set_params);
}
