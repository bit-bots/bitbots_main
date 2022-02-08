#include "bitbots_dynup/dynup_pywrapper.h"

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

/* Write a ROS message into a serialized string.
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

PyDynupWrapper::PyDynupWrapper(const std::string ns) : dynup_node_(std::make_shared<bitbots_dynup::DynupNode>(ns)){
}

void init_ros(std::string ns) {
    // remap clock
    std::map<std::string, std::string> remap = {{"/clock", "/" + ns + "clock"}};
    ros::init(remap, "dynup", ros::init_options::AnonymousName);
}

void spin_once() {
    ros::spinOnce();
}

moveit::py_bindings_tools::ByteString PyDynupWrapper::step(double dt,
                                                          const std::string &imu_msg,
                                                          const std::string &jointstate_msg) {
    std::string result =
            to_python<bitbots_msgs::JointCommand>(dynup_node_->step(dt,
                                                          from_python<sensor_msgs::Imu>(imu_msg),
                                                          from_python<sensor_msgs::JointState>(jointstate_msg)));
    return moveit::py_bindings_tools::serializeMsg(result);
}

moveit::py_bindings_tools::ByteString PyDynupWrapper::step_open_loop(double dt) {
    std::string result = to_python<geometry_msgs::PoseArray>(dynup_node_->step_open_loop(dt));
    return moveit::py_bindings_tools::serializeMsg(result);
}

void PyDynupWrapper::reset() {
    dynup_node_->reset();
}

void PyDynupWrapper::special_reset(double time) {
    dynup_node_->reset(time);
}

void PyDynupWrapper::set_engine_goal(std::string direction) {
    bitbots_dynup::DynupPoses poses = dynup_node_->getCurrentPoses();
    if (!poses.header.stamp.sec == 0) {
        DynupRequest request;
        request.direction = direction;
        request.l_foot_pose = poses.l_leg_pose;
        request.r_foot_pose = poses.r_leg_pose;
        request.l_hand_pose = poses.l_arm_pose;
        request.r_hand_pose = poses.r_arm_pose;
        dynup_node_->getIK()->setDirection(request.direction);
        dynup_node_->getEngine()->setGoals(request);
    }
}

int PyDynupWrapper::get_direction() {
    return dynup_node_->getEngine()->getDirection();
}

moveit::py_bindings_tools::ByteString PyDynupWrapper::get_poses() {
    bitbots_dynup::DynupPoses poses = dynup_node_->getCurrentPoses();
    return moveit::py_bindings_tools::serializeMsg(poses);
}

bool string2bool(const std::string &v) {
    return !v.empty() &&
           (strcasecmp(v.c_str(), "true") == 0 ||
            atoi(v.c_str()) != 0);
}

void PyDynupWrapper::set_node_dyn_reconf(const boost::python::object params) {
    using namespace boost::python;
    ROS_ERROR_STREAM("params" << params);
    extract<dict> cppdict_ext(params);
    if (!cppdict_ext.check()) {
        throw std::runtime_error(
                "PassObj::pass_dict: type error: not a python dict.");
    }

    dict cppdict = cppdict_ext();
    list keylist = cppdict.keys();

    // create dyn reconf object
    bitbots_dynup::DynUpConfig dyn_conf;

    // fill all values from dict to dyn reconf
    int const len = boost::python::len(keylist);
    // since c++ has no reflection we have to do this in a bad way
    for (int i = 0; i < len; ++i) {
        // operator[] is in python::boost::object
        std::string keystr = extract<std::string>(str(keylist[i]));
        std::string valstr = extract<std::string>(str(cppdict[keylist[i]]));
        if (keystr == "engine_rate") {
            dyn_conf.engine_rate = stof(valstr);
        } else if (keystr == "arm_extended_length") {
            dyn_conf.arm_extended_length = stof(valstr);
        } else if (keystr == "foot_distance") {
            dyn_conf.foot_distance = stof(valstr);
        } else if (keystr == "trunk_x_final") {
            dyn_conf.trunk_x_final = stof(valstr);
        } else if (keystr == "hand_walkready_pitch") {
            dyn_conf.hand_walkready_pitch = stof(valstr);
        } else if (keystr == "hand_walkready_height") {
            dyn_conf.hand_walkready_height = stof(valstr);
        } else if (keystr == "trunk_height") {
            dyn_conf.trunk_height = stof(valstr);
        } else if (keystr == "trunk_pitch") {
            dyn_conf.trunk_pitch = stof(valstr);
        } else if (keystr == "time_walkready") {
            dyn_conf.time_walkready = stof(valstr);
        } else if (keystr == "rise_time") {
            dyn_conf.rise_time = stof(valstr);
        } else if (keystr == "descend_time") {
            dyn_conf.descend_time = stof(valstr);
        } else if (keystr == "arm_side_offset_back") {
            dyn_conf.arm_side_offset_back = stof(valstr);
        } else if (keystr == "leg_min_length_back") {
            dyn_conf.leg_min_length_back = stof(valstr);
        } else if (keystr == "hands_behind_back_x") {
            dyn_conf.hands_behind_back_x = stof(valstr);
        } else if (keystr == "hands_behind_back_z") {
            dyn_conf.hands_behind_back_z = stof(valstr);
        } else if (keystr == "trunk_height_back") {
            dyn_conf.trunk_height_back = stof(valstr);
        } else if (keystr == "com_shift_1") {
            dyn_conf.com_shift_1 = stof(valstr);
        } else if (keystr == "com_shift_2") {
            dyn_conf.com_shift_2 = stof(valstr);
        } else if (keystr == "foot_angle") {
            dyn_conf.foot_angle = stof(valstr);
        } else if (keystr == "trunk_overshoot_angle_back") {
            dyn_conf.trunk_overshoot_angle_back = stof(valstr);
        } else if (keystr == "arms_angle_back") {
            dyn_conf.arms_angle_back = stof(valstr);
        } else if (keystr == "time_legs_close") {
            dyn_conf.time_legs_close = stof(valstr);
        } else if (keystr == "time_foot_ground_back") {
            dyn_conf.time_foot_ground_back = stof(valstr);
        } else if (keystr == "time_full_squat_hands") {
            dyn_conf.time_full_squat_hands = stof(valstr);
        } else if (keystr == "time_full_squat_legs") {
            dyn_conf.time_full_squat_legs = stof(valstr);
        } else if (keystr == "wait_in_squat_back") {
            dyn_conf.wait_in_squat_back = stof(valstr);
        } else if (keystr == "arm_side_offset_front") {
            dyn_conf.arm_side_offset_front = stof(valstr);
        } else if (keystr == "leg_min_length_front") {
            dyn_conf.leg_min_length_front = stof(valstr);
        } else if (keystr == "trunk_x_front") {
            dyn_conf.trunk_x_front = stof(valstr);
        } else if (keystr == "max_leg_angle") {
            dyn_conf.max_leg_angle = stof(valstr);
        } else if (keystr == "trunk_overshoot_angle_front") {
            dyn_conf.trunk_overshoot_angle_front = stof(valstr);
        } else if (keystr == "hands_pitch") {
            dyn_conf.hands_pitch = stof(valstr);
        } else if (keystr == "time_hands_side") {
            dyn_conf.time_hands_side = stof(valstr);
        } else if (keystr == "time_hands_rotate") {
            dyn_conf.time_hands_rotate = stof(valstr);
        } else if (keystr == "time_foot_close") {
            dyn_conf.time_foot_close = stof(valstr);
        } else if (keystr == "time_hands_front") {
            dyn_conf.time_hands_front = stof(valstr);
        } else if (keystr == "time_foot_ground_front") {
            dyn_conf.time_foot_ground_front = stof(valstr);
        } else if (keystr == "time_torso_45") {
            dyn_conf.time_torso_45 = stof(valstr);
        } else if (keystr == "time_to_squat") {
            dyn_conf.time_to_squat = stof(valstr);
        } else if (keystr == "wait_in_squat_front") {
            dyn_conf.wait_in_squat_front = stof(valstr);
        } else if (keystr == "stabilizing") {
            dyn_conf.stabilizing = string2bool(valstr);
        } else if (keystr == "minimal_displacement") {
            dyn_conf.minimal_displacement = string2bool(valstr);
        } else if (keystr == "stable_threshold") {
            dyn_conf.stable_threshold = stof(valstr);
        } else if (keystr == "stable_duration") {
            dyn_conf.stable_duration = stof(valstr);
        } else if (keystr == "stabilization_timeout") {
            dyn_conf.stabilization_timeout = stof(valstr);
        } else if (keystr == "display_debug") {
            dyn_conf.display_debug = string2bool(valstr);
        } else {
            std::cout << keystr << " not known. WILL BE IGNORED\n";
        }
    }
    dynup_node_->reconfigureCallback(dyn_conf, 0);
}


BOOST_PYTHON_MODULE(py_dynup)
        {
                using namespace boost::python;
                using namespace bitbots_dynup;

                class_<PyDynupWrapper>("PyDynupWrapper", init<std::string>())
                .def("step", &PyDynupWrapper::step)
                .def("step_open_loop", &PyDynupWrapper::step_open_loop)
                .def("reset", &PyDynupWrapper::reset)
                .def("special_reset", &PyDynupWrapper::special_reset)
                .def("set_node_dyn_reconf",
                &PyDynupWrapper::set_node_dyn_reconf)
                .def("get_poses", &PyDynupWrapper::get_poses)
                .def("get_direction", &PyDynupWrapper::get_direction)
                .def("set_engine_goal", &PyDynupWrapper::set_engine_goal);
                def("init_ros", &init_ros);
                def("spin_once", &spin_once);
        }