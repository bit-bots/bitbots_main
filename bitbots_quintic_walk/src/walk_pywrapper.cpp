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

void init_ros(){
    std::map<std::string, std::string> empty;
   ros::init(empty, "walking", ros::init_options::AnonymousName);
}

PyWalkWrapper::PyWalkWrapper(const std::string ns): walk_node_(std::make_shared<bitbots_quintic_walk::WalkNode>(ns))
{
  //walk_node_.reset(new );
  //ros::init(empty, "walking");
  //
  //walk_node_.reset(new bitbots_quintic_walk::WalkNode());
  set_robot_state(0);
}

moveit::py_bindings_tools::ByteString PyWalkWrapper::step(double dt, const std::string &cmdvel_msg, const std::string &imu_msg, const std::string &jointstate_msg){
  std::string result = to_python<bitbots_msgs::JointCommand>(walk_node_->step(dt, from_python<geometry_msgs::Twist>(cmdvel_msg), from_python<sensor_msgs::Imu>(imu_msg), from_python<sensor_msgs::JointState>(jointstate_msg)));
  return  moveit::py_bindings_tools::serializeMsg(result);
  }

void PyWalkWrapper::reset() {
  walk_node_->reset();
}

float PyWalkWrapper::get_phase(){
    return walk_node_->getEngine()->getPhase();
}

float PyWalkWrapper::get_freq(){
    return walk_node_->getEngine()->getFreq();
}

void PyWalkWrapper::set_robot_state(int state) {
  humanoid_league_msgs::RobotControlState state_msg;
  state_msg.state = state;
  walk_node_->robStateCb(state_msg);
}

void PyWalkWrapper::set_engine_dyn_reconf(const boost::python::object params){
    using namespace boost::python;

    extract< dict > cppdict_ext(params);
    if(!cppdict_ext.check()){
        throw std::runtime_error(
            "PassObj::pass_dict: type error: not a python dict.");
    }

    dict cppdict = cppdict_ext();
    list keylist = cppdict.keys();

    // create dyn reconf object
    bitbots_quintic_walk::bitbots_quintic_walk_engine_paramsConfig dyn_conf;

    // fill all values from dict to dyn reconf
    int const len = boost::python::len(keylist);
    //std::cout << "len(keylist) = " << len << std::endl;
    // since c++ has no reflection we have to do this in a bad way
    for(int i = 0; i < len; ++i){
        // operator[] is in python::boost::object
        std::string keystr = extract< std::string >(str(keylist[i]));
        std::string valstr = extract< std::string >(str(cppdict[keylist[i]]));
        //std::cout << "key:[" << keystr << "]->[" << valstr << "]" << std::endl;
        if (keystr == "double_support_ratio"){
            dyn_conf.double_support_ratio = std::stof(valstr);
        }else if (keystr == "freq"){
            dyn_conf.freq = std::stof(valstr);
        }else if (keystr == "foot_apex_phase"){
            dyn_conf.foot_apex_phase = std::stof(valstr);
        }else if (keystr == "foot_distance"){
            dyn_conf.foot_distance = std::stof(valstr);
        }else if (keystr == "foot_overshoot_phase"){
            dyn_conf.foot_overshoot_phase = std::stof(valstr);
        }else if (keystr == "foot_overshoot_ratio"){
            dyn_conf.foot_overshoot_ratio = std::stof(valstr);
        }else if (keystr == "foot_put_down_phase"){
            dyn_conf.foot_put_down_phase = std::stof(valstr);
        }else if (keystr == "foot_rise"){
            dyn_conf.foot_rise = std::stof(valstr);
        }else if (keystr == "foot_z_pause"){
            dyn_conf.foot_z_pause = std::stof(valstr);
        }else if (keystr == "trunk_height"){
            dyn_conf.trunk_height = std::stof(valstr);
        }else if (keystr == "trunk_pause"){
            dyn_conf.trunk_pause = std::stof(valstr);
        }else if (keystr == "trunk_phase"){
            dyn_conf.trunk_phase = std::stof(valstr);
        }else  if (keystr == "trunk_pitch"){
            dyn_conf.trunk_pitch = std::stof(valstr);
        }else if (keystr == "trunk_swing"){
            dyn_conf.trunk_swing = std::stof(valstr);
        }else if (keystr == "trunk_x_offset"){
            dyn_conf.trunk_x_offset = std::stof(valstr);
        }else if (keystr == "trunk_y_offset"){
            dyn_conf.trunk_y_offset = std::stof(valstr);
        }else if (keystr == "first_step_swing_factor"){
            dyn_conf.first_step_swing_factor = std::stof(valstr);
        }else if (keystr == "first_step_trunk_phase"){
            dyn_conf.first_step_trunk_phase = std::stof(valstr);
        }else if (keystr == "trunk_x_offset_p_coef_forward"){
            dyn_conf.trunk_x_offset_p_coef_forward = std::stof(valstr);
        }else if (keystr == "trunk_x_offset_p_coef_turn"){
            dyn_conf.trunk_x_offset_p_coef_turn = std::stof(valstr);
        }else if (keystr == "trunk_pitch_p_coef_forward"){
            dyn_conf.trunk_pitch_p_coef_forward = std::stof(valstr);
        }else if (keystr == "trunk_pitch_p_coef_turn"){
            dyn_conf.trunk_pitch_p_coef_turn = std::stof(valstr);
        }else{
            std::cout << keystr << " not knowen. WILL BE IGNORED\n";
        }
    }

    bitbots_quintic_walk::WalkEngine *engine = walk_node_->getEngine();
    engine->reconfCallback(dyn_conf, 0);
}

bool string2bool(const std::string & v){
    return !v.empty () &&
        (strcasecmp (v.c_str (), "true") == 0 ||
         atoi (v.c_str ()) != 0);
}

void PyWalkWrapper::set_node_dyn_reconf(const boost::python::object params){
    using namespace boost::python;

    extract< dict > cppdict_ext(params);
    if(!cppdict_ext.check()){
        throw std::runtime_error(
            "PassObj::pass_dict: type error: not a python dict.");
    }

    dict cppdict = cppdict_ext();
    list keylist = cppdict.keys();

    // create dyn reconf object
    bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig dyn_conf;

    // fill all values from dict to dyn reconf
    int const len = boost::python::len(keylist);
    //std::cout << "len(keylist) = " << len << std::endl;
    // since c++ has no reflection we have to do this in a bad way
    for(int i = 0; i < len; ++i){
        // operator[] is in python::boost::object
        std::string keystr = extract< std::string >(str(keylist[i]));
        std::string valstr = extract< std::string >(str(cppdict[keylist[i]]));
        //std::cout << "key:[" << keystr << "]->[" << valstr << "]" << std::endl;
        if (keystr == "debug_active"){
            dyn_conf.debug_active = string2bool(valstr);
        }else if (keystr == "engine_freq"){
            dyn_conf.engine_freq = string2bool(valstr);
        }else if (keystr == "odom_pub_factor"){
            dyn_conf.odom_pub_factor = std::stof(valstr);
        }else if (keystr == "ik_timeout"){
            dyn_conf.ik_timeout = std::stof(valstr);
        }else if (keystr == "pressure_phase_reset_active"){
            dyn_conf.pressure_phase_reset_active = string2bool(valstr);
        }else if (keystr == "ground_min_pressure"){
            dyn_conf.ground_min_pressure = std::stof(valstr);
        }else if (keystr == "phase_reset_phase"){
            dyn_conf.phase_reset_phase = std::stof(valstr);
        }else if (keystr == "joint_min_effort"){
            dyn_conf.joint_min_effort = std::stof(valstr);
        }else if (keystr == "effort_phase_reset_active"){
            dyn_conf.effort_phase_reset_active = string2bool(valstr);
        }else if (keystr == "phase_rest_active"){
            dyn_conf.phase_rest_active = string2bool(valstr);
        }else if (keystr == "pause_duration"){
            dyn_conf.pause_duration = std::stof(valstr);
        }else if (keystr == "imu_active"){
            dyn_conf.imu_active = string2bool(valstr);
        }else  if (keystr == "imu_pitch_threshold"){
            dyn_conf.imu_pitch_threshold = std::stof(valstr);
        }else if (keystr == "imu_roll_threshold"){
            dyn_conf.imu_roll_threshold = std::stof(valstr);
        }else if (keystr == "imu_pitch_vel_threshold"){
            dyn_conf.imu_pitch_vel_threshold = std::stof(valstr);
        }else if (keystr == "imu_roll_vel_threshold"){
            dyn_conf.imu_roll_vel_threshold = std::stof(valstr);
        }else if (keystr == "max_step_x"){
            dyn_conf.max_step_x = std::stof(valstr);
        }else if (keystr == "max_step_y"){
            dyn_conf.max_step_y = std::stof(valstr);
        }else if (keystr == "max_step_xy"){
            dyn_conf.max_step_xy = std::stof(valstr);
        }else if (keystr == "max_step_z"){
            dyn_conf.max_step_z = std::stof(valstr);
        }else if (keystr == "max_step_angular"){
            dyn_conf.max_step_angular = std::stof(valstr);
        }else{
            std::cout << keystr << " not knowen. WILL BE IGNORED\n";
        }
    }

    walk_node_->reconfCallback(dyn_conf, 0);
}


BOOST_PYTHON_MODULE(py_quintic_walk)
{
    using namespace boost::python;
    using namespace bitbots_quintic_walk;

    class_<PyWalkWrapper>("PyWalkWrapper", init<std::string>())
        .def("step", &PyWalkWrapper::step)
        .def("set_robot_state", &PyWalkWrapper::set_robot_state)
        .def("reset", &PyWalkWrapper::reset)
        .def("set_engine_dyn_reconf",
             &PyWalkWrapper::set_engine_dyn_reconf)
        .def("set_node_dyn_reconf",
             &PyWalkWrapper::set_node_dyn_reconf)
        .def("get_phase", &PyWalkWrapper::get_phase)
        .def("get_freq", &PyWalkWrapper::get_freq);

    def("init_ros", &init_ros);
}
