from typing import Optional

import numpy as np
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped transform support
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_geometry_msgs import PointStamped

from bitbots_rl_motion.handlers import Handler
from bitbots_rl_motion.handlers.tf_utils import shared_tf_buffer


class BallHandler(Handler):
    """Provides the ball as the policy observed it during training.

    The AMP kick task observes the position of the ball in the frame of an anchor body
    of the robot, which is the torso. How the ball moves is only given to the critic
    during training, so the policy itself does not need a ball velocity.
    """

    def __init__(self, node):
        self._node = node

        self._anchor_frame = str(node.get_parameter("ball.anchor_frame").value)
        self._timeout = float(node.get_parameter("ball.timeout").value)
        self._max_covariance = float(node.get_parameter("ball.max_covariance").value)

        self._tf_buffer = shared_tf_buffer(self._node)

        self._ball: Optional[PoseWithCovarianceStamped] = None

        self._node.create_subscription(
            PoseWithCovarianceStamped,
            str(node.get_parameter("ball.topic").value),
            self._ball_callback,
            1,
        )

    def has_data(self) -> bool:
        # Non-blocking: the policy is only started once a kick is requested, and a
        # missing ball is reported by is_available() instead of stalling the node.
        return True

    def is_available(self) -> bool:
        """Whether we know where the ball is well enough to kick it.

        The ball filter keeps publishing an estimate after it lost the ball, with a
        growing covariance, so how old the message is does not tell us whether the
        estimate is still worth anything. Both are checked.
        """
        if self._ball is None:
            return False
        age = (self._node.get_clock().now() - Time.from_msg(self._ball.header.stamp)).nanoseconds / 1e9
        if not 0.0 <= age <= self._timeout:
            return False
        # The covariance is a 6x6 matrix as a flat array, so 0 is x and 7 is y
        covariance = self._ball.pose.covariance
        return bool(covariance[0] < self._max_covariance and covariance[7] < self._max_covariance)

    def get_position(self) -> Optional[np.ndarray]:
        """Position of the ball in the anchor frame, or None if it can not be determined"""
        if self._ball is None:
            return None
        point = PointStamped()
        point.header.frame_id = self._ball.header.frame_id
        # Ask for the newest transform instead of the one at the stamp of the ball,
        # as the ball estimate is published in a frame that does not move with us
        point.header.stamp = Time().to_msg()
        point.point = self._ball.pose.pose.position
        try:
            transformed = self._tf_buffer.transform(point, self._anchor_frame, timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self._node.get_logger().warn(
                f"Could not transform the ball into '{self._anchor_frame}': {e}", throttle_duration_sec=1.0
            )
            return None
        return np.array(
            [transformed.point.x, transformed.point.y, transformed.point.z],
            dtype=np.float32,
        )

    def _ball_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._ball = msg
