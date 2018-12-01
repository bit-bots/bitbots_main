#!/usr/bin/env python
"""
This is the ROS-Node which contains the head behavior, starts the appropriate DSD, initializes the HeadBlackboard
and subscribes to head_behavior specific ROS-Topics.
"""
import os

import rospy

from bitbots_connector.blackboard import HeadBlackboard
from bitbots_dsd.dsd import DSD

from humanoid_league_msgs.msg import HeadMode as HeadModeMsg
from bitbots_ros_control.msg import JointCommand


def run(dsd):
    """
    Main run-loop
    :returns: Never
    """
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        dsd.update()
        rate.sleep()



def init():
    """
    Initialize new components needed for head_behavior:
    blackboard, dsd, rostopic subscriber
    """
    blackboard = HeadBlackboard()

    rospy.Subscriber('/head_mode', HeadModeMsg, blackboard.head_capsule.head_mode_callback, queue_size=1)
    blackboard.head_capsule.position_publisher = rospy.Publisher("head_motor_goals", JointCommand, queue_size=10)

    dirname = os.path.dirname(os.path.realpath(__file__))

    dsd = DSD(blackboard, '/debug/dsd/head_behavior')
    dsd.register_actions(os.path.join(dirname, 'actions'))
    dsd.register_decisions(os.path.join(dirname, 'decisions'))
    dsd.load_behavior(os.path.join(dirname, 'head_behavior.dsd'))

    rospy.init_node(blackboard.config['rosnode'])

    return dsd


if __name__ == '__main__':
    run(init())
