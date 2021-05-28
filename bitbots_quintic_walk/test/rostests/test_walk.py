#!/usr/bin/env python3
import time

import rospy
from bitbots_msgs.msg import JointCommand
from geometry_msgs.msg import Twist

from bitbots_test.mocks import MockSubscriber
from bitbots_test.test_case import WebotsTestCase


class TestWalk(WebotsTestCase):

    def test_start(self):
        """ test if node starts correctly without warnings/errors/criticals
        and if it is still there after some time (did not crash by itself)"""
        # wait to make sure node is up
        time.sleep(1)
        self.assertNotNegativeRosLogs()

    def test_no_joint_goals(self):
        """test if joint goals are published when walking is activated and only then"""
        # setup
        sub = MockSubscriber("DynamixelController/command", JointCommand, tcp_nodelay=True)

        # execution
        # wait some time
        sub.wait_until_connected()
        time.sleep(1)

        # verification
        sub.assertNothingReceived()

    def test_joint_goals(self):
        # setup
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        sub = MockSubscriber("DynamixelController/command", JointCommand, tcp_nodelay=True)

        # execution
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        pub.publish(cmd_vel)

        # verification
        # make sure something is published
        sub.assertMessageReceived()

        # destruct
        cmd_vel.linear.x = 0.0
        pub.publish(cmd_vel)
        rospy.sleep(1)

    def test_walk(self):
        """test if the walking is really moving the robot around in the simulation"""
        # setup
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        start_pose = self.get_robot_pose()

        # execution
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        pub.publish(cmd_vel)
        rospy.sleep(10)

        # verification
        # should have moved away
        self.assertRobotNotPosition(start_pose.position, threshold=0.5)
        # but still standing
        self.assertRobotStanding()

        # destruct
        cmd_vel.linear.x = 0.0
        pub.publish(cmd_vel)
        rospy.sleep(1)

    def test_speed(self):
        """test if the robot actually walks with the correct commanded speed"""
        pass

    def test_walk_odometry(self):
        """test if the walk odometry is correct"""
        pass


class DummyTest(WebotsTestCase):

    def test_nothing(self):
        self.assertInRange(0.5, 0, 1)


if __name__ == "__main__":
    from bitbots_test import run_rostests
    run_rostests(DummyTest)
