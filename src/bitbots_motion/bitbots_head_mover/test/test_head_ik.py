# File for testing head ik
import time

import numpy
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion
from rclpy.action import ActionClient
from rclpy.node import Node

from bitbots_msgs.action import LookAt
from bitbots_msgs.msg import HeadMode

yaw_limits = [-1.43, 1.43]
pitch_limits = [-1.23, 1.01]


class TestHeadIK(Node):
    """Node to control the wolfgang humanoid."""

    def __init__(self, node_name: str):
        super().__init__("test_head_ik")
        self._look_action_client = ActionClient(self, LookAt, "look_at_goal")
        self._head_mode_publisher = self.create_publisher(HeadMode, "head_mode", 10)
        self._ball_pub = self.create_publisher(PoseWithCovarianceStamped, "ball_position_relative_filtered", 1)

    def covariance_list(self):
        covariance = numpy.empty(36, dtype=numpy.float64)
        covariance[0] = 1.0
        covariance[6] = 2.0
        covariance[30] = 3.0
        covariance[1] = 4.0
        covariance[7] = 5.0
        covariance[31] = 6.0
        covariance[5] = 7.0
        covariance[11] = 8.0
        covariance[35] = 9.0

    def pose_with_covariance(self, x, y, z=0.0):
        point = Point(x=x, y=y, z=z)
        quat = Quaternion(x=0.2, y=0.3, z=0.5, w=0.8)
        pose = PoseWithCovariance(pose=Pose(position=point, orientation=quat))
        pose.covariance = self.covariance_list()

        return PoseWithCovarianceStamped(pose=pose)

    def execute_looking_behavior(self):
        ball_msg = self.pose_with_covariance(x=8.0, y=9.0)
        self._ball_pub.publish(ball_msg)
        self._head_mode_publisher.publish(HeadMode("TRACK_BALL"))

        while True:
            time.sleep(2)
            self._look_action_client.send_goal(yaw_limits[0], 0)
            time.sleep(2)
            self._look_action_client.send_goal(0, pitch_limits[0])
            time.sleep(2)
            self._look_action_client.send_goal(yaw_limits[1], 0)
            time.sleep(2)
            self._look_action_client.send_goal(0, pitch_limits[1])
