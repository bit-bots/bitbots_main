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

PyDynupWrapper::PyDynupWrapper(const std::string ns) {
    dynup_node_ = std::make_shared<bitbots_dynup::DynupNode>(ns);
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
    std::string result =
            to_python<bitbots_msgs::JointCommand>(dynup_node_->step(dt));
    return moveit::py_bindings_tools::serializeMsg(result);
}

void PyDynupWrapper::reset() {
    dynup_node_->reset();
}

void PyDynupWrapper::special_reset(double time) {
    dynup_node_->reset(time);
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
            dyn_conf.engine_rate = string2bool(valstr);
        } else if (keystr == "arm_extended_length") {
            dyn_conf.arm_extended_length = string2bool(valstr);
        } else if (keystr == "foot_distance") {
            dyn_conf.foot_distance = string2bool(valstr);
        } else if (keystr == "trunk_x_final") {
            dyn_conf.trunk_x_final = string2bool(valstr);
        } else if (keystr == "hand_walkready_pitch") {
            dyn_conf.hand_walkready_pitch = string2bool(valstr);
        } else if (keystr == "hand_walkready_height") {
            dyn_conf.hand_walkready_height = string2bool(valstr);
        } else if (keystr == "trunk_height") {
            dyn_conf.trunk_height = string2bool(valstr);
        } else if (keystr == "trunk_pitch") {
            dyn_conf.trunk_pitch = string2bool(valstr);
        } else if (keystr == "time_walkready") {
            dyn_conf.time_walkready = string2bool(valstr);
        } else if (keystr == "rise_time") {
            dyn_conf.rise_time = string2bool(valstr);
        } else if (keystr == "descend_time") {
            dyn_conf.descend_time = string2bool(valstr);
        } else if (keystr == "arm_side_offset_back") {
            dyn_conf.arm_side_offset_back = string2bool(valstr);
        } else if (keystr == "leg_min_length_back") {
            dyn_conf.leg_min_length_back = string2bool(valstr);
        } else if (keystr == "hands_behind_back_x") {
            dyn_conf.hands_behind_back_x = string2bool(valstr);
        } else if (keystr == "hands_behind_back_z") {
            dyn_conf.hands_behind_back_z = string2bool(valstr);
        } else if (keystr == "trunk_height_back") {
            dyn_conf.trunk_height_back = string2bool(valstr);
        } else if (keystr == "com_shift_1") {
            dyn_conf.com_shift_1 = string2bool(valstr);
        } else if (keystr == "com_shift_2") {
            dyn_conf.com_shift_2 = string2bool(valstr);
        } else if (keystr == "foot_angle") {
            dyn_conf.foot_angle = string2bool(valstr);
        } else if (keystr == "trunk_overshoot_angle_back") {
            dyn_conf.trunk_overshoot_angle_back = string2bool(valstr);
        } else if (keystr == "arms_angle_back") {
            dyn_conf.arms_angle_back = string2bool(valstr);
        } else if (keystr == "time_legs_close") {
            dyn_conf.time_legs_close = string2bool(valstr);
        } else if (keystr == "time_foot_ground_back") {
            dyn_conf.time_foot_ground_back = string2bool(valstr);
        } else if (keystr == "time_full_squat_hands") {
            dyn_conf.time_full_squat_hands = string2bool(valstr);
        } else if (keystr == "time_full_squat_legs") {
            dyn_conf.time_full_squat_legs = string2bool(valstr);
        } else if (keystr == "wait_in_squat_back") {
            dyn_conf.wait_in_squat_back = string2bool(valstr);
        } else if (keystr == "arm_side_offset_front") {
            dyn_conf.arm_side_offset_front = string2bool(valstr);
        } else if (keystr == "leg_min_length_front") {
            dyn_conf.leg_min_length_front = string2bool(valstr);
        } else if (keystr == "trunk_x_front") {
            dyn_conf.trunk_x_front = string2bool(valstr);
        } else if (keystr == "max_leg_angle") {
            dyn_conf.max_leg_angle = string2bool(valstr);
        } else if (keystr == "trunk_overshoot_angle_front") {
            dyn_conf.trunk_overshoot_angle_front = string2bool(valstr);
        } else if (keystr == "hands_pitch") {
            dyn_conf.hands_pitch = string2bool(valstr);
        } else if (keystr == "time_hands_side") {
            dyn_conf.time_hands_side = string2bool(valstr);
        } else if (keystr == "time_hands_rotate") {
            dyn_conf.time_hands_rotate = string2bool(valstr);
        } else if (keystr == "time_foot_close") {
            dyn_conf.time_foot_close = string2bool(valstr);
        } else if (keystr == "time_hands_front") {
            dyn_conf.time_hands_front = string2bool(valstr);
        } else if (keystr == "time_foot_ground_front") {
            dyn_conf.time_foot_ground_front = string2bool(valstr);
        } else if (keystr == "time_torso_45") {
            dyn_conf.time_torso_45 = string2bool(valstr);
        } else if (keystr == "time_to_squat") {
            dyn_conf.time_to_squat = string2bool(valstr);
        } else if (keystr == "wait_in_squat_front") {
            dyn_conf.wait_in_squat_front = string2bool(valstr);
        } else if (keystr == "stabilizing") {
            dyn_conf.stabilizing = string2bool(valstr);
        } else if (keystr == "minimal_displacement") {
            dyn_conf.minimal_displacement = string2bool(valstr);
        } else if (keystr == "stable_threshold") {
            dyn_conf.stable_threshold = string2bool(valstr);
        } else if (keystr == "stable_duration") {
            dyn_conf.stable_duration = string2bool(valstr);
        } else if (keystr == "stabilization_timeout") {
            dyn_conf.stabilization_timeout = string2bool(valstr);
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
                .def("get_direction", &PyDynupWrapper::get_direction);

                def("init_ros", &init_ros);
                def("spin_once", &spin_once);
        }