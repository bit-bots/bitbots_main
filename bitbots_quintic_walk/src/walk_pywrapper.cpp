//
// Created by nfiedler on 6/13/20.
//

#include "bitbots_quintic_walk/walk_pywrapper.h"


/* Read a ROS message from a serialized string.
  */
template <typename M>
M from_python(const std::string &str_msg)
{
  size_t serial_size = str_msg.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  for (size_t i = 0; i < serial_size; ++i)
  {
    buffer[i] = str_msg[i];
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  M msg;
  ros::serialization::Serializer<M>::read(stream, msg);
  return msg;
}

/* Write a ROS message into a serialized string.
*/
template <typename M>
std::string to_python(const M& msg)
{
  size_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, msg);
  std::string str_msg;
  str_msg.reserve(serial_size);
  for (size_t i = 0; i < serial_size; ++i)
  {
    str_msg.push_back(buffer[i]);
  }
  return str_msg;
}

PyWalk::PyWalk() {
  std::map<std::string, std::string> empty;
  ros::init(empty, "PyWalking");
  //
  walk_node_.reset(new bitbots_quintic_walk::WalkNode());
  set_robot_state(0);
}

std::string PyWalk::step(double dt, const std::string &cmdvel_msg, const std::string &imu_msg, const std::string &jointstate_msg){
    return to_python<bitbots_msgs::JointCommand>(walk_node_->step(dt, from_python<geometry_msgs::Twist>(cmdvel_msg), from_python<sensor_msgs::Imu>(imu_msg), from_python<sensor_msgs::JointState>(jointstate_msg)));
  }

void PyWalk::reset() {
  walk_node_->reset();
}

void PyWalk::set_robot_state(int state) {
  humanoid_league_msgs::RobotControlState state_msg;
  state_msg.state = state;
  walk_node_->robStateCb(state_msg);
}

BOOST_PYTHON_MODULE(py_quintic_walk)
{
    using namespace boost::python;
    using namespace bitbots_quintic_walk;

    class_<PyWalk>("PyWalkWrapper")
        .def("step", &PyWalk::step)
        .def("set_robot_state", &PyWalk::set_robot_state)
        .def("reset", &PyWalk::reset);
}
