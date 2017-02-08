#!/usr/bin/env python3
import rospy
import time
from bitbots_common.pose.pypose import PyPose as Pose
from trajectory_msgs.msg import JointTrajectory
import numpy as np
from bitbots_common.util.pose_to_message import pose_to_traj_msg

rospy.init_node("sine", anonymous=False)
robot_type_name = rospy.get_param("/robot_type_name")
used_motor_cids = rospy.get_param("/cm730/" + robot_type_name + "/motors")
#used_motor_names = Pose().get_joint_names_cids(used_motor_cids)
used_motor_names = ['RShoulderPitch', 'LShoulderPitch', 'RShoulderRoll', 'LShoulderRoll', 'RElbow', 'LElbow', 'RHipYaw',
                    'LHipYaw', 'RHipRoll', 'LHipRoll', 'RHipPitch', 'LHipPitch', 'RKnee', 'LKnee', 'RAnklePitch',
                    'LAnklePitch', 'RAnkleRoll', 'LAnkleRoll', 'HeadPan', 'HeadTilt']
used_motor_names = [x.encode("utf8") for x in used_motor_names]

rospy.logerr(used_motor_names)
pose = Pose()

joint_goal_publisher = rospy.Publisher('/motion_motor_goals', JointTrajectory, queue_size=1)

rate = rospy.Rate(200)
start_time = time.time()
speed = 0.5

while not rospy.is_shutdown():
    factor = (time.time() - start_time) * speed
    pos = (np.math.pi * factor) % (2*np.math.pi) - np.math.pi
    pos = np.math.degrees(pos)
    pose.set_positions(["LShoulderPitch".encode()], [pos])
    msg = pose_to_traj_msg(pose, used_motor_names)
    #rospy.logwarn(pose.get_joint_by_name("LShoulderPitch".encode()).get_position())
    joint_goal_publisher.publish(msg)
    rate.sleep()

#todo testen indem ich die werte direkt im cm node generiere und nicht per ros transmitte, wenn dann noch ruckelt, liegts nich an ros