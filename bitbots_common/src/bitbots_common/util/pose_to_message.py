import math
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


def pose_to_traj_msg(pose, motor_names, joint_traj_msg, traj_point_msg):
    traj_point_msg.positions = pose.get_positions_rad_names(motor_names)
    traj_point_msg.velocities = pose.get_speeds_names(motor_names)
    traj_point_msg.time_from_start = rospy.Duration.from_sec(0.05)
    # make an array with String objects (ros message type)
    joint_traj_msg.points = []
    joint_traj_msg.points.append(traj_point_msg)
    joint_traj_msg.header.stamp = rospy.Time.now()
    return joint_traj_msg


def pose_goal_to_traj_msg(pose, motor_names, joint_traj_msg, traj_point_msg):
    traj_point_msg.positions = pose.get_goal_rad_names(motor_names)
    traj_point_msg.velocities = pose.get_speeds_names(motor_names)
    traj_point_msg.time_from_start = rospy.Duration.from_sec(0.05)
    # make an array with String objects (ros message type)
    joint_traj_msg.points = []
    joint_traj_msg.points.append(traj_point_msg)
    joint_traj_msg.header.stamp = rospy.Time.now()
    return joint_traj_msg
