from typing import Optional

import numpy as np
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped/Vector3Stamped transform support
import tf2_ros
from bitbots_tf_buffer import Buffer
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Vector3Stamped
from rclpy.duration import Duration
from rclpy.time import Time

from bitbots_rl_motion.handlers import Handler
from bitbots_rl_motion.history_buffer import HistoryBuffer


class DribbleCommandHandler(Handler):
    """Provides the dribble policy's ball and kick command observations.

    The ball pose (``ball_position_relative_filtered``, stamped in odom) and
    the dribble command (``rl_dribble_command``) are both transformed into the
    policy's base frame with tf every control step, so they stay correct while
    the robot moves regardless of the frame they were published in.

    The dribble command is a single ``Vector3Stamped``: the desired ball
    velocity after the kick, i.e. kick direction times target ball speed in
    m/s, expressed in ``header.frame_id``. The policy observes the normalized
    direction and the speed separately.

    A short ball history provides the policy's second, older ball observation
    (it was trained with a past ball position to be able to estimate ball
    motion).
    """

    def __init__(self, node):
        self._node = node

        self._base_frame = str(node.declare_parameter("dribble.base_frame", "base_link").value)
        # Number of control steps between the current and the past ball
        # observation; must match the offset used during training.
        self._ball_prev_offset = int(node.declare_parameter("dribble.ball_prev_offset", 5).value)

        self._tf_buffer = Buffer(node=self._node)

        self._ball_pose: Optional[PoseWithCovarianceStamped] = None
        self._command: Optional[Vector3Stamped] = None

        # Last successfully transformed values, held on transient tf misses so
        # the observation does not jump.
        self._last_ball_b: Optional[np.ndarray] = None
        self._last_dir_b = np.array([1.0, 0.0], dtype=np.float32)
        self._speed = 0.0

        self._ball_history = HistoryBuffer(self._ball_prev_offset + 1)

        self._node.create_subscription(
            PoseWithCovarianceStamped, "ball_position_relative_filtered", self._ball_callback, 1
        )
        self._node.create_subscription(Vector3Stamped, "rl_dribble_command", self._command_callback, 1)

    def _ball_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._ball_pose = msg

    def _command_callback(self, msg: Vector3Stamped) -> None:
        self._command = msg

    def has_data(self) -> bool:
        # Non-blocking: the dribble node additionally gates activation on
        # has_ball_and_command() so it never runs on missing data.
        return True

    def has_ball_and_command(self) -> bool:
        return self._ball_pose is not None and self._command is not None

    def update(self) -> None:
        """Advance the per-step state; call exactly once per control step."""
        ball_b = self._transform_ball()
        if ball_b is not None:
            self._last_ball_b = ball_b
        if self._last_ball_b is not None:
            self._ball_history.append(self._last_ball_b)

        command_b = self._transform_command()
        if command_b is not None:
            speed = float(np.linalg.norm(command_b))
            if speed > 1e-6:
                self._last_dir_b = command_b / speed
                self._speed = speed

    def reset(self) -> None:
        """Restart the ball history so it re-saturates with fresh data."""
        self._ball_history.reset()
        self._last_ball_b = None

    def _transform_ball(self) -> Optional[np.ndarray]:
        if self._ball_pose is None:
            return None
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self._ball_pose.header.frame_id
        pose_stamped.header.stamp = Time(seconds=0).to_msg()
        pose_stamped.pose = self._ball_pose.pose.pose
        try:
            transformed = self._tf_buffer.transform(pose_stamped, self._base_frame, timeout=Duration(seconds=0.1))
            return np.array([transformed.pose.position.x, transformed.pose.position.y], dtype=np.float32)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self._node.get_logger().warning(
                f"Ball transform to {self._base_frame} failed: {e}", throttle_duration_sec=1.0
            )
            return None

    def _transform_command(self) -> Optional[np.ndarray]:
        if self._command is None:
            return None
        vector_stamped = Vector3Stamped()
        vector_stamped.header.frame_id = self._command.header.frame_id
        vector_stamped.header.stamp = Time(seconds=0).to_msg()
        vector_stamped.vector = self._command.vector
        try:
            transformed = self._tf_buffer.transform(vector_stamped, self._base_frame, timeout=Duration(seconds=0.1))
            return np.array([transformed.vector.x, transformed.vector.y], dtype=np.float32)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self._node.get_logger().warning(
                f"Dribble command transform to {self._base_frame} failed: {e}", throttle_duration_sec=1.0
            )
            return None

    def get_ball_obs(self) -> np.ndarray:
        """Current ball position in the base frame."""
        # Zeros until the first successful ball transform after a reset (e.g.
        # a transient tf miss right at activation); the policy was trained
        # with heavy ball observation noise and tolerates this briefly.
        if self._last_ball_b is None:
            return np.zeros(2, dtype=np.float32)
        return self._ball_history.buffer[-1]

    def get_ball_prev_obs(self) -> np.ndarray:
        """Ball position from ball_prev_offset control steps ago."""
        if self._last_ball_b is None:
            return np.zeros(2, dtype=np.float32)
        return self._ball_history.buffer[0]

    def get_kick_dir_obs(self) -> np.ndarray:
        """Unit kick direction in the base frame."""
        return self._last_dir_b

    def get_kick_speed(self) -> float:
        """Target ball speed in m/s."""
        return self._speed
