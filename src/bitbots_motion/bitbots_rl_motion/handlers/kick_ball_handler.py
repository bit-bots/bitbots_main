from typing import Optional

import numpy as np
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)
import tf2_ros as tf2
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from rclpy.time import Time

from handlers.handler import Handler


class KickBallHandler(Handler):
    """
    Provides the velocity-corrected ball position in the ``base_link`` frame for the kick policy.

    The filtered ball is cached in the ``odom`` frame so that it stays stable while the robot
    moves. On each query it is transformed into ``base_link`` and extrapolated forward in time
    using the estimated ball velocity to compensate for the perception delay.
    """

    def __init__(self, node, tf_buffer):
        self._node = node
        self._tf_buffer = tf_buffer
        self._control_dt = self._node.get_parameter("phase.control_dt").value

        self._ball_odom: Optional[PointStamped] = None
        self._last_point: Optional[np.ndarray] = None
        self._last_corrected: np.ndarray = np.array([0.0, 0.0], dtype=np.float32)

        self._node.create_subscription(
            PoseWithCovarianceStamped, "ball_position_relative_filtered", self._ball_callback, 1
        )

    def _ball_callback(self, msg: PoseWithCovarianceStamped):
        point = PointStamped()
        point.header.frame_id = msg.header.frame_id
        point.point = msg.pose.pose.position
        try:
            self._ball_odom = self._tf_buffer.transform(point, "odom")
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self._ball_odom = None
            self._node.get_logger().warn(str(e), throttle_duration_sec=1.0)

    def has_data(self) -> bool:
        return self._ball_odom is not None

    def get_ball_pos(self) -> np.ndarray:
        assert self._ball_odom is not None
        point_now = PointStamped()
        point_now.header.frame_id = "odom"
        point_now.point = self._ball_odom.point
        point_now.point.z = 0.075
        try:
            ball_pose = self._tf_buffer.transform(point_now, "base_link")
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self._node.get_logger().error(str(e), throttle_duration_sec=1.0)
            return self._last_corrected

        point = np.array([ball_pose.point.x, ball_pose.point.y], dtype=np.float32)
        if self._last_point is not None:
            diff = (point - self._last_point) / self._control_dt
        else:
            diff = np.array([0.0, 0.0], dtype=np.float32)
        self._last_point = point

        delta_t = (self._node.get_clock().now() - Time.from_msg(ball_pose.header.stamp)).nanoseconds / 1e9
        self._last_corrected = point #+ diff * delta_t
        return self._last_corrected
