from typing import Optional

import numpy as np
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped transform support
import tf2_ros
from bitbots_tf_buffer import Buffer
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_geometry_msgs import PointStamped

from bitbots_rl_motion.handlers import Handler


class BallHandler(Handler):
    """Gives the ball position in the frame of an anchor body of the robot.

    The Pi Plus AMP kick policy observes the 3D ball position in the torso
    frame (full rotation, not only yaw). The training clips each axis to a
    maximum value, so this handler does the same.
    """

    def __init__(self, node, tf_buffer: Buffer):
        self._node = node
        self._tf_buffer = tf_buffer

        self._anchor_frame = str(node.get_parameter("ball.anchor_frame").value)
        self._timeout = float(node.get_parameter("ball.timeout").value)
        self._max_covariance = float(node.get_parameter("ball.max_covariance").value)
        self._position_clip = float(node.get_parameter("ball.position_clip").value)

        self._ball: Optional[PoseWithCovarianceStamped] = None

        self._node.create_subscription(
            PoseWithCovarianceStamped,
            str(node.get_parameter("ball.topic").value),
            self._ball_callback,
            1,
        )

    def _ball_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._ball = msg

    def has_data(self) -> bool:
        # Do not block the node. The kick starts only with a kick goal, and
        # is_available() tells if the ball is known.
        return True

    def is_available(self) -> bool:
        """Returns True if the ball estimate is recent and its covariance is small."""
        if self._ball is None:
            return False
        age = (self._node.get_clock().now() - Time.from_msg(self._ball.header.stamp)).nanoseconds / 1e9
        if not 0.0 <= age <= self._timeout:
            return False
        # The covariance is a flat 6x6 matrix. Index 0 is x, index 7 is y.
        covariance = self._ball.pose.covariance
        return bool(covariance[0] < self._max_covariance and covariance[7] < self._max_covariance)

    def get_position(self) -> Optional[np.ndarray]:
        """Returns the ball position in the anchor frame, or None if the transform fails."""
        if self._ball is None:
            return None
        point = PointStamped()
        point.header.frame_id = self._ball.header.frame_id
        # Use the latest transform. The ball estimate is in a world-fixed frame.
        point.header.stamp = Time().to_msg()
        point.point = self._ball.pose.pose.position
        try:
            transformed = self._tf_buffer.transform(point, self._anchor_frame, timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self._node.get_logger().warning(
                f"Could not transform the ball into '{self._anchor_frame}': {e}", throttle_duration_sec=1.0
            )
            return None
        position = np.array([transformed.point.x, transformed.point.y, transformed.point.z], dtype=np.float32)
        return np.clip(position, -self._position_clip, self._position_clip)
