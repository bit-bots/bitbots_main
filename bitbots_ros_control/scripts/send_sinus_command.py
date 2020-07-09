#!/usr/bin/env python3

import rospy
import math

from bitbots_msgs.msg import JointCommand


DYNAMIXEL_CMD_TOPIC = "/DynamixelController/command"
JOINT_NAME = "LAnkleRoll"
PUBLISH_RATE = 1000

# sin function
FREQUENCY = 0.5
AMPLITUDE = 72 #degree

if __name__ == "__main__":
    msg = JointCommand(
        joint_names=[JOINT_NAME],
        velocities=[-1],
        accelerations=[-1],
        max_currents=[-1])

    rospy.init_node("send_sinus_command")
    pub = rospy.Publisher(DYNAMIXEL_CMD_TOPIC, JointCommand, queue_size=1)

    rate = rospy.Rate(PUBLISH_RATE)
    while not rospy.is_shutdown():
        time = rospy.Time.now()
        position = math.radians(AMPLITUDE) * math.sin(2 * math.pi * FREQUENCY * time.to_sec())

        msg.header.stamp = time
        msg.positions=[position]
        pub.publish(msg)
        rate.sleep()