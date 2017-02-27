#!/usr/bin/env python3
import rospy
from bitbots_common.connector.connector import HeadConnector
from bitbots_head_behaviour.decisions.head_duty_decider import HeadDutyDecider
from bitbots_stackmachine.stack_machine_module import StackMachineModule
from humanoid_league_msgs.msg import HeadMode, BallRelative, ObstacleRelative
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class HeadNode(StackMachineModule):
    def __init__(self):
        super().__init__()
        self.connector = HeadConnector()
        self.connector.config = rospy.get_param("Behaviour")

        self.connector.head.position_publisher = rospy.Publisher("head_motor_goals", JointTrajectory, queue_size=10)

        rospy.Subscriber("joint_states", JointState, self.connector.head.joint_state_cb)
        rospy.Subscriber("head_duty", HeadMode, self.connector.head.cb_headmode, queue_size=10)
        rospy.Subscriber("ball_relative", BallRelative, self.connector.vision.ball_callback)
        rospy.Subscriber("obstacle_relative", ObstacleRelative, self.connector.vision.obstacle_callback)

        self.set_start_module(HeadDutyDecider)
        rospy.init_node("Headbehaviour")

    def run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

if __name__ == "__main__":
    hb = HeadNode()
    hb.run()

