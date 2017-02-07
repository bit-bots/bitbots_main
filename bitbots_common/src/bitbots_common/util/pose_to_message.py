import math
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


def pose_to_traj_msg(pose, motor_names):
    msg = JointTrajectoryPoint()
    msg.positions = pose.get_positions_rad_names(motor_names)
    msg.velocities = pose.get_speeds_names(motor_names)
    traj_msg = JointTrajectory()
    # make an array with String objects (ros message type)
    traj_msg.joint_names = motor_names
    traj_msg.points = []
    traj_msg.points.append(msg)
    traj_msg.header.stamp = rospy.Time.now()
    return traj_msg

def pose_goal_to_traj_msg(pose, motor_names):
    msg = JointTrajectoryPoint()
    msg.positions = pose.get_goal_rad_names(motor_names)
    msg.velocities = pose.get_speeds_names(motor_names)
    traj_msg = JointTrajectory()
    # make an array with String objects (ros message type)
    traj_msg.joint_names = motor_names
    traj_msg.points = []
    traj_msg.points.append(msg)
    traj_msg.header.stamp = rospy.Time.now()
    return traj_msg