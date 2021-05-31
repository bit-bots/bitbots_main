#!/usr/bin/env python3
import time

import rospy
from bitbots_msgs.msg import JointCommand, SupportState
from geometry_msgs.msg import Twist, Point, Pose, Quaternion

from bitbots_test.mocks import MockSubscriber
from bitbots_test.test_case import WebotsTestCase, RosNodeTestCase
from nav_msgs.msg import Odometry


class TestWalk(WebotsTestCase):

    def test_start(self):
        """ test if node starts correctly without warnings/errors/criticals
        and if it is still there after some time (did not crash by itself)"""
        # wait to make sure node is up
        sub = MockSubscriber("DynamixelController/command", JointCommand, tcp_nodelay=True)
        sub.wait_until_connected()
        time.sleep(2)

        self.assertNoNegativeRosLogs(node="walking")

    def test_no_joint_goals(self):
        """test if joint goals are published when walking is activated and only then"""
        # setup
        sub = MockSubscriber("DynamixelController/command", JointCommand, tcp_nodelay=True)
        sub.wait_until_connected()

        # execution
        # wait some time
        time.sleep(1)

        # verification
        sub.assertNothingReceived()

    def test_joint_goals(self):
        # setup
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        sub = MockSubscriber("DynamixelController/command", JointCommand, tcp_nodelay=True)
        sub.wait_until_connected()
        # wait for support state to make sure the walking is started
        sub_support_state = MockSubscriber("walk_support_state", SupportState, tcp_nodelay=True)
        sub_support_state.wait_until_connected()
        time.sleep(10)

        # execution
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        pub.publish(cmd_vel)
        rospy.sleep(5)

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
        self.set_ball_position(Point(10, 0, 0))
        self.set_robot_pose(Pose(Point(0, 0, 0.42), Quaternion(0, 0, 0, 1)))
        # let robot walk a bit to get into walkready
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        pub.publish(cmd_vel)
        rospy.sleep(1)
        cmd_vel.linear.x = 0.0
        pub.publish(cmd_vel)
        rospy.sleep(5)
        self.set_robot_pose(Pose(Point(0, 0, 0.42), Quaternion(0, 0, 0, 1)))

        # execution
        cmd_vel.linear.x = 0.1
        pub.publish(cmd_vel)
        rospy.sleep(10)
        cmd_vel = Twist()
        pub.publish(cmd_vel)
        rospy.sleep(2)

        # verification
        # should have moved away
        self.assertRobotNotPosition(Point(0, 0, 0), threshold=0.5)
        # but still standing
        self.assertRobotStanding()

    def test_walk_odometry(self):
        """test if the walk odometry is correct"""
        # setup
        current_odom = None

        def odom_cb(msg):
            nonlocal current_odom
            current_odom = msg

        sub = MockSubscriber("walk_engine_odometry", Odometry, odom_cb, tcp_nodelay=True)
        sub.wait_until_connected()
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.set_ball_position(Point(10, 0, 0))
        self.set_robot_pose(Pose(Point(0, 0, 0.42), Quaternion(0, 0, 0, 1)))

        # execution
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        cmd_vel.linear.y = 0.05
        cmd_vel.angular.z = 0.1
        pub.publish(cmd_vel)
        rospy.sleep(10)
        cmd_vel = Twist()
        pub.publish(cmd_vel)
        rospy.sleep(2)

        # verification
        # robot should be at odom position
        self.assertRobotPose(current_odom.pose.pose)
        self.assertRobotStanding()

    def test_speed(self):
        """test if the robot actually walks with the correct commanded speed"""
        pass  # todo


if __name__ == "__main__":
    from bitbots_test import run_rostests

    run_rostests(TestWalk)
