from typing import Optional

import numpy as np
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped transform support
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.time import Time

from handlers.handler import Handler

from bitbots_tf_buffer import Buffer, TransformListener


class BallHandler(Handler):
    def __init__(self, node):
        self._node = node

        self._ball_pose: Optional[PoseWithCovarianceStamped] = None

        self._tf_buffer = Buffer(node=node)
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._ball_pos_sub = node.create_subscription(
            PoseWithCovarianceStamped, "ball_position_relative_filtered", self._ball_pos_callback, 10
        )

    def _ball_pos_callback(self, msg: PoseWithCovarianceStamped):
        self._ball_pose = msg

    def has_data(self):
        return self._ball_pose is not None

    def get_ball_pos(self) -> np.ndarray:
        assert self._ball_pose is not None
        pose_stamped = PoseStamped()
        # Use Time(0) to get the latest available transform regardless of message age
        pose_stamped.header.frame_id = self._ball_pose.header.frame_id
        pose_stamped.header.stamp = Time(seconds=0).to_msg()
        pose_stamped.pose = self._ball_pose.pose.pose
        try:
            transformed = self._tf_buffer.transform(pose_stamped, "base_footprint", timeout=Duration(seconds=0.1))
            return np.array([transformed.pose.position.x, transformed.pose.position.y], dtype=np.float32)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self._node.get_logger().warn(f"TF transform to base_footprint failed: {e}", throttle_duration_sec=1.0)
            pos = self._ball_pose.pose.pose.position
            return np.array([pos.x, pos.y], dtype=np.float32)
