#include <iostream>
#include <fstream>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <stdexcept>

#include "kinematic_joint.hpp"
#include "kinematic_robot.hpp"

// Forward declaration
Robot::Kinematics::KRobot* create_robot(YAML::Node config);
bool file_exists(const char* fileName);

namespace Robot {
namespace Kinematics {

#define INDIRECTION(X,Y) Y(X)
#define STR(X) #X
// PROJECT_DIR and INSTALL_DIR are variables defined in the project main CMakeLists.txt file
const std::string default_file = std::string(INDIRECTION(PROJECT_DIR, STR)) + std::string("/share/bitbots/config/darwin.yaml");
const std::string default_installed_file = std::string(INDIRECTION(INSTALL_DIR, STR)) + std::string("/share/bitbots/config/darwin.yaml");
#undef STR
#undef INDIRECTION
/**
 * Loads and creates a valid Robot representation from a configuration file
 */
KRobot* load_robot_from_file(const std::string& file) {
    YAML::Node config = file_exists(file.data())? YAML::LoadFile(file.data()): YAML::Load(file.data());
    KRobot * robot = create_robot(config);
    assert(robot->all_data_valid());
    return robot;
}

} } // namespace Robot Kinematics

bool file_exists(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}

using namespace Robot;
using namespace Kinematics;

/**
 * Init the joint from yaml config and return the name
 */
std::string init_joint_from_node(YAML::Node config, KJoint& output) {
    int id = config["id"].as<int>();

    Eigen::Vector3d transform, rotations, angles;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > masses;
    std::string id_string = std::string("Current Joint ") + std::to_string(id) + std::string(" has malformed config.");

    double mass;
    double off_x, off_y, off_z;
    YAML::Node offsets = config["mass_offsets"];

    try {
        for(YAML::Node::const_iterator it = offsets.begin(); it != offsets.end(); ++it) {
            std::string part_string = it->first.as<std::string>();
            mass = it->second["mass"].as<double>();
            off_x = it->second["offset"][0].as<double>();
            off_y = it->second["offset"][1].as<double>();
            off_z = it->second["offset"][2].as<double>();
            masses.push_back(Eigen::Vector4d(off_x / 1000.0, off_y / 1000.0, off_z / 1000.0, mass));
        }
    } catch(std::runtime_error& e) {
        std::cout<<e.what()<<std::endl;
        throw std::runtime_error(std::string(e.what()) + std::string("\nError while filling offsets") + id_string);
    }

    double x, y, z, r, p, yaw;
    try {
        x = config["transform"][0].as<double>();
        y = config["transform"][1].as<double>();
        z = config["transform"][2].as<double>();
        transform = Eigen::Vector3d(x, y, z) / 1000;
        r = config["rpy"][0].as<double>();
        p = config["rpy"][1].as<double>();
        yaw = config["rpy"][2].as<double>();
        rotations = Eigen::Vector3d(r, p, yaw);
    } catch(std::runtime_error& e) {
        std::cout<<e.what()<<std::endl;
        throw std::runtime_error(std::string(e.what()) + std::string("\nError while creating transform") + id_string);
    }

    int def, min , max;
    try {
        def = 0;//config["def_min_max_angles"][0].as<int>();
        min = 0;//config["def_min_max_angles"][0].as<int>();
        max = 180;//config["def_min_max_angles"][0].as<int>();
        //angles = Vector3d(0, -180, 180)
        angles = Eigen::Vector3d(def, min, max);
    } catch(std::runtime_error& e) {
        std::cout<<e.what()<<std::endl;
        throw std::runtime_error(std::string(e.what()) + std::string("\nError while setting angles") + id_string);
    }

    output = KJoint(transform, rotations, masses, angles, id);
    return config["name"].as<std::string>();
}


KRobot* create_robot(YAML::Node config) {
    // Fill the joint name id Mapping over the whole process
    KRobot::IdMapping joint_mapping;
    //cdef dict robot
    YAML::Node& robot = config;
    YAML::Node joint_specialization = config["JointOverride"];
    //YAML::Node joints_angles = config["joints"];// Joint angles, but not in this cpp version
    const int num_joints = 37;
    std::vector<::KJoint, Eigen::aligned_allocator<::KJoint> > joints(num_joints);
    int max_id = robot["max_joint_id"].as<int>();

    //#add default angles to root config, because the angles are provided in a extra config file
    //robot["Root"]["def_min_max_angles"] = [0, 0, 0]
    //#insert Root _Joint
    joint_mapping[init_joint_from_node(config["Root"], joints[0])] = 0;
    YAML::Node chains = robot["ChainNames"];
    //#insert all joints in dict
    //cdef dict limits
    for(YAML::Node::const_iterator it = chains.begin(); it != chains.end(); ++it){
        YAML::Node chain = config[it->as<std::string>()];
        //for joint in robot[str(chain)]:
        for(YAML::Node::const_iterator it = chain.begin(); it != chain.end(); ++it){
            YAML::Node joint = *it;
            joint_mapping[init_joint_from_node(joint, joints[joint["id"].as<int>()])] = joint["id"].as<int>();
        }
    }
    //#Adding "Virtual joints to the jointvector, those that are unused by the robot"
    YAML::Node virt = robot["Virtual"];
    for(YAML::Node::const_iterator it = virt.begin(); it != virt.end(); ++it) {
        YAML::Node joint = *it;
        //set_joint_config_limits(joint, joints_angles, max_id)
        joint_mapping[init_joint_from_node(joint, joints[joint["id"].as<int>()])] = joint["id"].as<int>();
    }

    //#init the chain template with joint ids
    KRobot::ChainsTemplate chain_template = std::vector<std::vector<int> >();
    std::vector<int> chain_ids;
    for(YAML::Node::const_iterator it = chains.begin(); it != chains.end(); ++it) {
        YAML::Node chain = config[it->as<std::string>()];
        chain_ids = std::vector<int>();
        for(YAML::Node::iterator it = chain.begin(); it != chain.end(); ++it){
            YAML::Node joint = *it;
            chain_ids.push_back(joint["id"].as<int>());
        }
        chain_template.push_back(std::vector<int>(chain_ids));
    }
    //#init _Joint Id Mapping
    // Done when creating a joint
    //for(const KJoint& joint: joints) {
    //    joint_mapping[joint.get_name()] = joint.get_id();
    //}
    //init Chain Id Mapping
    KRobot::ChainMapping chain_mapping;
    int idx = 0;
    for(YAML::Node::const_iterator it = chains.begin(); it != chains.end(); ++it) {
        chain_mapping[it->as<std::string>()] = idx;
        ++idx;
    }
    for(auto it: joint_mapping) {
        assert(it.second < num_joints);
    }
    return new KRobot(joint_mapping, chain_mapping, joints, chain_template, max_id);
}
