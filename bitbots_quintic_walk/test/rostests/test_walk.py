#!/usr/bin/env python3
import math
import time

import rospy
from biped_interfaces.msg import Phase
from bitbots_msgs.msg import JointCommand
from geometry_msgs.msg import Twist, Point, Pose, Quaternion

from bitbots_test.mocks import MockSubscriber
from bitbots_test.test_case import WebotsTestCase
from nav_msgs.msg import Odometry

walkready = JointCommand(
    joint_names=["HeadPan", "HeadTilt", "LShoulderPitch", "LShoulderRoll", "LElbow", "RShoulderPitch",
                 "RShoulderRoll", "RElbow"],
    velocities=[5.0] * 8,
    accelerations=[-1.0] * 8,
    max_currents=[-1.0] * 8,
    positions=[
        0.0,  # HeadPan
        0.0,  # HeadTilt
        math.radians(75.27),  # LShoulderPitch
        0.0,  # LShoulderRoll
        math.radians(35.86),  # LElbow
        math.radians(-75.58),  # RShoulderPitch
        0.0,  # RShoulderRoll
        math.radians(-36.10),  # RElbow
    ]
)


class TestWalk(WebotsTestCase):

    def initialize_everything(self):
        # get arms walkready
        joint_pub = rospy.Publisher("DynamixelController/command", JointCommand, queue_size=1)
        joint_pub.publish(walkready)

        # start and stop walking to get walkready
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        pub.publish(cmd_vel)
        rospy.sleep(1)
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.x = -1
        pub.publish(cmd_vel)
        rospy.sleep(1)

        # remove ball
        self.set_ball_position(Point(10, 0, 0))

        # reset robot to start
        self.set_robot_pose(Pose(Point(0, 0, 0.42), Quaternion(0, 0, 0, 1)))

    def test_start(self):
        """ test if node starts correctly without warnings/errors/criticals
        and if it is still there after some time (did not crash by itself)"""
        # setup
        self.initialize_everything()
        # wait to make sure node is up
        sub = MockSubscriber("DynamixelController/command", JointCommand, tcp_nodelay=True)
        sub.wait_until_connected()

        # execution
        time.sleep(2)

        # verification
        self.assertNoNegativeRosLogs(node="walking")

    def test_no_joint_goals(self):
        """test if joint goals are published when walking is activated and only then"""
        # setup
        self.initialize_everything()
        sub = MockSubscriber("DynamixelController/command", JointCommand, tcp_nodelay=True)
        sub.wait_until_connected()

        # execution
        # wait some time
        time.sleep(1)

        # verification
        sub.assertNothingReceived()

    def test_joint_goals(self):
        # setup
        self.initialize_everything()
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        sub = MockSubscriber("DynamixelController/command", JointCommand, tcp_nodelay=True)
        sub.wait_until_connected()
        # wait for support state to make sure the walking is started
        sub_support_state = MockSubscriber("walk_support_state", Phase, tcp_nodelay=True)
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

    def test_walk(self):
        """test if the walking is really moving the robot around in the simulation"""
        self.initialize_everything()
        # setup
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # execution
        cmd_vel = Twist()
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
        """
        Test if the walk odometry is correct.
        This means also that the robot is walking with the correct speed.
        """

        def odom_cb(msg):
            nonlocal current_odom
            current_odom = msg

        # setup
        self.initialize_everything()
        current_odom = None
        sub = MockSubscriber("walk_engine_odometry", Odometry, odom_cb, tcp_nodelay=True)
        sub.wait_until_connected()
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # remember start odom pose
        rospy.sleep(1)
        start_odom = current_odom

        # execution
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        cmd_vel.linear.y = 0.05
        cmd_vel.angular.z = 0.1
        pub.publish(cmd_vel)
        rospy.sleep(10)
        cmd_vel = Twist()
        cmd_vel.angular.x = -1
        pub.publish(cmd_vel)
        rospy.sleep(2)

        # verification
        end_odom = current_odom
        odom_diff = Pose()
        odom_diff.position.x = end_odom.pose.pose.position.x - start_odom.pose.pose.position.x
        odom_diff.position.y = end_odom.pose.pose.position.y - start_odom.pose.pose.position.y
        odom_diff.position.z = end_odom.pose.pose.position.z - start_odom.pose.pose.position.z
        # difference between quaternions is annoying, so just use end pose, other tests did not turn
        odom_diff.orientation.w = end_odom.pose.pose.orientation.w
        odom_diff.orientation.x = end_odom.pose.pose.orientation.x
        odom_diff.orientation.y = end_odom.pose.pose.orientation.y
        odom_diff.orientation.z = end_odom.pose.pose.orientation.z
        # robot should be at odom position
        self.assertRobotPose(odom_diff, lin_threshold=1, ang_threshold=1)
        self.assertRobotStanding()


if __name__ == "__main__":
    from bitbots_test import run_rostests

    run_rostests(TestWalk)
