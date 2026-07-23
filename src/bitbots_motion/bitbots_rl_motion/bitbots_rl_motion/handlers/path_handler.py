import math
from typing import Optional

import numpy as np
from bitbots_tf_buffer import Buffer
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool

from bitbots_rl_motion.handlers import Handler


def _wrap_to_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _heading_from_quat_zw(z: float, w: float) -> float:
    """Heading of an xyzw quaternion, in the convention used during training.

    mjlab reads the robot heading as ``wrap(2 * atan2(qz, qw))``, i.e. from the
    yaw part of the quaternion alone. For an upright pose that is the usual yaw;
    for the tilted torso of a walking robot it differs slightly from the Euler
    yaw, so the same formula is used here to keep the observation in
    distribution.
    """
    return _wrap_to_pi(2.0 * math.atan2(z, w))


def _resample_by_arc_length(points: np.ndarray, spacing: float) -> np.ndarray:
    """Resample a polyline to points evenly spaced by ``spacing`` of arc length.

    Mirrors the training-time path generation, which walks the reference path in
    fixed arc-length steps (``np.arange(0, total_length, spacing)``), so the last
    sample can fall up to one spacing short of the goal.
    """
    if len(points) < 2:
        return points.copy()

    # Drop repeated poses; they would make the arc length non-monotonic.
    segments = np.linalg.norm(np.diff(points, axis=0), axis=1)
    points = points[np.concatenate(([True], segments > 1e-9))]
    if len(points) < 2:
        return points.copy()

    segments = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc_length = np.concatenate(([0.0], np.cumsum(segments)))
    targets = np.arange(0.0, arc_length[-1], spacing)
    return np.stack(
        [np.interp(targets, arc_length, points[:, 0]), np.interp(targets, arc_length, points[:, 1])],
        axis=1,
    )


class PathHandler(Handler):
    """Builds the path-following policy's command observation from a ROS path.

    The policy was trained on reference paths sampled uniformly along their arc
    length (``point_spacing`` meters between consecutive samples). Its command
    term is

      * the ``num_observations`` path samples starting at the sample closest to
        the robot, ``observation_spacing`` samples apart (i.e.
        ``observation_spacing * point_spacing`` meters of arc length), expressed
        in the robot's base frame and flattened to ``[x0, y0, x1, y1, ...]``,
      * followed by the goal orientation as ``[cos(yaw_err), sin(yaw_err)]``,
        where ``yaw_err`` is the wrapped difference between the orientation of
        the last pose of the path and the robot's heading.

    Sample indices past the end of the path are clamped to the last sample, which
    is how training padded its fixed-size path tensors: the observation saturates
    on the goal as the robot arrives.

    An incoming ``nav_msgs/Path`` is therefore resampled to the training spacing
    on receipt. A path published in a frame other than ``path.frame`` is
    transformed into it once, at receipt (paths are usually republished
    continuously, so the snapshot stays fresh). What moves the observation while
    the robot walks is the robot pose inside ``path.frame``, which is looked up
    from tf at the control rate.
    """

    def __init__(self, node):
        self._node = node

        # The frame the path is held (and the robot pose is looked up) in.
        self._frame = str(node.declare_parameter("path.frame", "odom").value)
        # Training transforms the path into the root link frame, projected to the
        # ground plane, so the base link (not the base footprint) is used here.
        self._base_frame = str(node.declare_parameter("path.base_frame", "base_link").value)
        topic = str(node.declare_parameter("path.topic", "path").value)

        # Observation layout; must match the trained policy.
        self._num_observations = int(node.declare_parameter("path.num_observations", 8).value)
        self._observation_spacing = int(node.declare_parameter("path.observation_spacing", 15).value)
        self._point_spacing = float(node.declare_parameter("path.point_spacing", 0.01).value)
        assert self._num_observations > 0, "path.num_observations must be positive"
        assert self._observation_spacing > 0, "path.observation_spacing must be positive"
        assert self._point_spacing > 0.0, "path.point_spacing must be positive"

        # Goal gate; matches PathCommandCfg.near_goal_(yaw_)threshold. Reaching it
        # is what freezes the gait, so these have to match training as well.
        self._near_goal_threshold = float(node.declare_parameter("path.near_goal_threshold", 0.3).value)
        self._near_goal_yaw_threshold = float(node.declare_parameter("path.near_goal_yaw_threshold", 0.3).value)
        # Age above which the robot pose counts as lost and the policy is stopped.
        self._max_pose_age = float(node.declare_parameter("path.max_pose_age", 0.5).value)

        self._tf_buffer = Buffer(node=node)

        # Path state, expressed in self._frame. None while no path is set.
        self._points: Optional[np.ndarray] = None  # (M, 2), spaced by point_spacing
        self._goal_xy: Optional[np.ndarray] = None  # (2,) position of the last pose
        self._goal_yaw = 0.0  # orientation of the last pose

        # Robot pose in self._frame, refreshed from tf at the control rate.
        self._pose: Optional[np.ndarray] = None  # (3,) [x, y, heading]
        self._pose_stamp: Optional[Time] = None

        # Derived state, rebuilt on every update().
        self._command = np.zeros(self._num_observations * 2 + 2, dtype=np.float32)
        self._lookahead = np.zeros((self._num_observations, 2))
        self._dist_to_goal = math.inf
        self._yaw_err = 0.0
        self._at_goal = False

        node.create_subscription(Path, topic, self._path_callback, 1)
        self._lookahead_pub = node.create_publisher(Path, "path_lookahead", 1)
        # Latched so a subscriber that starts late still learns the current state.
        self._goal_reached_pub = node.create_publisher(
            Bool, "path_goal_reached", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self._published_goal_reached: Optional[bool] = None

        # The pose (and with it the command) is refreshed on its own timer rather
        # than from obs(), so that it is already available when the control loop
        # asks whether this handler has data.
        node.create_timer(node.get_parameter("phase.control_dt").value, self.update)

    # ------------------------------------------------------------------ #
    # path input
    # ------------------------------------------------------------------ #
    def _path_callback(self, msg: Path) -> None:
        if not msg.poses:
            if self._points is not None:
                self._node.get_logger().info("Received an empty path, stopping path following.")
            self._clear_path()
            return

        points = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        goal_orientation = msg.poses[-1].pose.orientation
        goal_yaw = _heading_from_quat_zw(goal_orientation.z, goal_orientation.w)

        frame = msg.header.frame_id or self._frame
        if frame != self._frame:
            transformed = self._transform_to_path_frame(points, goal_yaw, frame, msg.header.stamp)
            if transformed is None:
                return  # Keep the previous path rather than following a wrong one.
            points, goal_yaw = transformed

        self._points = _resample_by_arc_length(points, self._point_spacing)
        self._goal_xy = points[-1]
        self._goal_yaw = goal_yaw

    def _clear_path(self) -> None:
        self._points = None
        self._goal_xy = None
        self._dist_to_goal = math.inf
        self._at_goal = False
        # Without a path there is no goal to be at, and update() stops running,
        # so the latched flag has to be corrected here.
        self._publish_goal_reached()

    def _transform_to_path_frame(
        self, points: np.ndarray, goal_yaw: float, frame: str, stamp
    ) -> Optional[tuple[np.ndarray, float]]:
        """Express a path given in ``frame`` in the configured path frame.

        The path is planar, so only the yaw and the x/y translation of the
        transform are applied.
        """
        try:
            transform = self._tf_buffer.lookup_transform(self._frame, frame, Time.from_msg(stamp))
        except Exception:
            # The stamp can be just outside the buffer; the latest transform
            # between two world frames is a good enough fallback.
            try:
                transform = self._tf_buffer.lookup_transform(self._frame, frame, Time())
            except Exception as e:
                self._node.get_logger().warning(
                    f"Could not transform the path from {frame} to {self._frame}: {e}",
                    throttle_duration_sec=1.0,
                )
                return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = _heading_from_quat_zw(rotation.z, rotation.w)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        rotated = np.stack(
            [
                cos_yaw * points[:, 0] - sin_yaw * points[:, 1] + translation.x,
                sin_yaw * points[:, 0] + cos_yaw * points[:, 1] + translation.y,
            ],
            axis=1,
        )
        return rotated, _wrap_to_pi(goal_yaw + yaw)

    # ------------------------------------------------------------------ #
    # per-step update
    # ------------------------------------------------------------------ #
    def update(self) -> None:
        """Refresh the robot pose from tf and rebuild the command observation."""
        self._update_pose()
        if self._points is None or self._pose is None:
            return
        self._update_command()
        self._publish_debug()

    def _update_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(self._frame, self._base_frame, Time())
        except Exception as e:
            self._node.get_logger().warning(
                f"Could not look up {self._base_frame} in {self._frame}: {e}",
                throttle_duration_sec=1.0,
            )
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self._pose = np.array(
            [translation.x, translation.y, _heading_from_quat_zw(rotation.z, rotation.w)],
        )
        self._pose_stamp = self._node.get_clock().now()

    def _update_command(self) -> None:
        assert self._points is not None and self._goal_xy is not None and self._pose is not None

        position, heading = self._pose[:2], float(self._pose[2])

        # The observed points start at the path sample closest to the robot. The
        # search is global, exactly as in training.
        offsets = self._points - position
        closest = int(np.argmin(np.einsum("ij,ij->i", offsets, offsets)))
        indices = np.clip(
            closest + np.arange(self._num_observations) * self._observation_spacing,
            0,
            len(self._points) - 1,
        )
        self._lookahead = self._points[indices]

        # Express them in the base frame (a planar rotation by -heading).
        relative = self._lookahead - position
        cos_heading, sin_heading = math.cos(heading), math.sin(heading)
        local = np.stack(
            [
                cos_heading * relative[:, 0] + sin_heading * relative[:, 1],
                -sin_heading * relative[:, 0] + cos_heading * relative[:, 1],
            ],
            axis=1,
        )

        self._dist_to_goal = float(np.linalg.norm(self._goal_xy - position))
        self._yaw_err = _wrap_to_pi(self._goal_yaw - heading)
        # Both position and orientation have to be reached before the gait is
        # frozen, so the robot keeps stepping while it turns onto the goal pose.
        self._at_goal = (
            self._dist_to_goal < self._near_goal_threshold and abs(self._yaw_err) < self._near_goal_yaw_threshold
        )

        self._command = np.concatenate([local.reshape(-1), [math.cos(self._yaw_err), math.sin(self._yaw_err)]]).astype(
            np.float32
        )

    def _publish_goal_reached(self) -> None:
        """Publish the goal state, but only when it changed."""
        if self._published_goal_reached != self._at_goal:
            self._goal_reached_pub.publish(Bool(data=self._at_goal))
            self._published_goal_reached = self._at_goal

    def _publish_debug(self) -> None:
        self._publish_goal_reached()

        if self._lookahead_pub.get_subscription_count() == 0:
            return
        path = Path()
        path.header.frame_id = self._frame
        path.header.stamp = self._node.get_clock().now().to_msg()
        for x, y in self._lookahead:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._lookahead_pub.publish(path)

    # ------------------------------------------------------------------ #
    # accessors
    # ------------------------------------------------------------------ #
    def has_data(self) -> bool:
        """True once a path and a recent robot pose inside its frame are known."""
        if self._points is None or self._pose_stamp is None:
            return False
        age = (self._node.get_clock().now() - self._pose_stamp).nanoseconds / 1e9
        return age <= self._max_pose_age

    def has_path(self) -> bool:
        return self._points is not None

    def get_command(self) -> np.ndarray:
        """Path points in the base frame plus the goal orientation."""
        return self._command

    def is_at_goal(self) -> bool:
        """True once the robot is on the goal pose (position and orientation)."""
        return self._at_goal

    def get_distance_to_goal(self) -> float:
        return self._dist_to_goal
