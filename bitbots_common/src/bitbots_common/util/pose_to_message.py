import math
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


def pose_to_traj_msg(pose):
    msg = JointTrajectoryPoint()
    msg.positions = pose.get_positions_rad()
    msg.velocities = pose.get_speeds()
    traj_msg = JointTrajectory()
    # make an array with String objects (ros message type)
    joints = []
    names = pose.get_joint_names()
    for joint in names:
        joints.append(joint)
    traj_msg.joint_names = joints
    traj_msg.points = []
    traj_msg.points.append(msg)
    traj_msg.header.stamp = rospy.Time.now()
    return traj_msg