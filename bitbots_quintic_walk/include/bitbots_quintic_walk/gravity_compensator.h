// Walking, 2018, Philipp Ruppel

#pragma once

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <urdf/model.h>
#include <urdf_model/model.h>

namespace bitbots_quintic_walk {

class GravityCompensator {
  struct PointMass {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double mass = 0;
    PointMass &operator+=(const PointMass &other) {
      center =
          (center * mass + other.center * other.mass) / (mass + other.mass);
      mass += other.mass;
      return *this;
    }
  };
  std::unordered_map<const moveit::core::LinkModel *, PointMass> link_masses;
  Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);
  std::vector<double> efforts, temp, child_forces, root_forces;

  PointMass compensateGravityLink(const moveit::core::RobotState &state,
                                  const moveit::core::LinkModel *link) {
    //ROS_INFO("l1");
    PointMass mass = link_masses[link];
    //ROS_INFO("%s", link->getName().c_str());
    mass.center = state.getGlobalLinkTransform(link) * mass.center;
    //ROS_INFO("l3");
    for (auto *joint : link->getChildJointModels()) {
      mass += compensateGravityJoint(state, joint);
    }
    //ROS_INFO("l4");
    return mass;
  }

  void applyMassToJoint(const moveit::core::RobotState &state,
                        const moveit::core::JointModel *joint,
                        const PointMass &mass) {
    Eigen::Affine3d joint_transform;
    joint->computeTransform(state.getVariablePositions() +
                                joint->getFirstVariableIndex(),
                            joint_transform);
    joint_transform =
        (joint->getParentLinkModel()
             ? state.getGlobalLinkTransform(joint->getParentLinkModel())
             : Eigen::Affine3d::Identity()) *
        joint->getChildLinkModel()->getJointOriginTransform() * joint_transform;
    if (auto *j =
            dynamic_cast<const moveit::core::RevoluteJointModel *>(joint)) {
      auto joint_axis = joint_transform.rotation() * j->getAxis();
      double effort = joint_axis.cross(gravity).dot(
                          mass.center - joint_transform.translation()) *
                      mass.mass;
      efforts[joint->getFirstVariableIndex()] += effort;
    }
  }

  PointMass compensateGravityJoint(const moveit::core::RobotState &state,
                                   const moveit::core::JointModel *joint) {
    //ROS_INFO("j1");
    PointMass mass = compensateGravityLink(state, joint->getChildLinkModel());
    //ROS_INFO("j2");
    applyMassToJoint(state, joint, mass);
    //ROS_INFO("j3");
    return mass;
  }

public:
  GravityCompensator(){}
  GravityCompensator(moveit::core::RobotModelPtr &robot_model) {
    efforts.resize(robot_model->getVariableCount());
    temp.resize(robot_model->getVariableCount());
    for (auto *link : robot_model->getLinkModels()) {
      link_masses[link] = PointMass();
      if (auto urdf_link = robot_model->getURDF()->getLink(link->getName())) {
        if (urdf_link->inertial) {
          link_masses[link].mass = urdf_link->inertial->mass;
          link_masses[link].center.x() = urdf_link->inertial->origin.position.x;
          link_masses[link].center.y() = urdf_link->inertial->origin.position.y;
          link_masses[link].center.z() = urdf_link->inertial->origin.position.z;
        }
      }
    }
  }
  void compensateGravity(
      moveit::core::RobotState &state,
      const std::vector<std::pair<std::string, double>> &contacts) {
    //ROS_INFO("g1");
    // state.update();

    for (size_t i = 0; i < state.getVariableCount(); i++) {
      efforts[i] = 0;
    }
    //ROS_INFO("g2");
    auto total_mass =
        compensateGravityJoint(state, state.getRobotModel()->getRootJoint());
    child_forces = efforts;
    //ROS_INFO("g3");
    for (size_t i = 0; i < state.getVariableCount(); i++) {
      efforts[i] = 0;
    }
    //ROS_INFO("g4");
    for (auto *joint_model : state.getRobotModel()->getJointModels()) {
      applyMassToJoint(state, joint_model, total_mass);
    }
    root_forces = efforts;
    //ROS_INFO("g5");
    for (size_t i = 0; i < state.getVariableCount(); i++) {
      temp[i] = 0;
    }
    //ROS_INFO("g6");
    for (auto &contact : contacts) {
      double weight = contact.second;
      for (auto *joint = state.getRobotModel()
                             ->getLinkModel(contact.first)
                             ->getParentJointModel();
           joint;
           joint = (joint->getParentLinkModel()
                        ? joint->getParentLinkModel()->getParentJointModel()
                        : nullptr)) {
        //ROS_INFO("g7");
        for (size_t variable_index = joint->getFirstVariableIndex();
             variable_index <
             joint->getFirstVariableIndex() + joint->getVariableCount();
             variable_index++) {
          //ROS_INFO("g8");
          temp[variable_index] += weight;
        }
      }
    }
    //ROS_INFO("g9");
    for (size_t i = 0; i < state.getVariableCount(); i++) {
      double effort =
          mix(child_forces[i], -root_forces[i] - child_forces[i], temp[i]);
      state.setVariableEffort(i, state.getVariableEffort(i) + effort);
    }
    //ROS_INFO("g10");
  }
};
}