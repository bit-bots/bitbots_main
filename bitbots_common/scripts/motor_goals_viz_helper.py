#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

class MotorVizHelper:
    def __init__(self):
        rospy.init_node("motor_viz_helper", log_level=rospy.DEBUG, anonymous=False)
        self.joint_state_msg = JointState()
        self.joint_publisher = rospy.Publisher('joint_states', JointState, queue_size=1)
        rospy.Subscriber("walking_motor_goals", JointTrajectory, self.cb, queue_size=1)
        rospy.spin()

    def cb(self, msg:JointTrajectory):
        self.joint_state_msg.header = msg.header
        self.joint_state_msg.name = msg.joint_names
        self.joint_state_msg.position = msg.points[0].positions
        self.joint_publisher.publish(self.joint_state_msg)

helper = MotorVizHelper()

