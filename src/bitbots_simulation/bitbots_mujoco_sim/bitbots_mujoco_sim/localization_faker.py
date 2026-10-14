import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from transforms3d.euler import euler2quat, quat2euler


def planar_transform(x: float, y: float, yaw: float) -> np.ndarray:
    """Build a homogeneous 2D transformation matrix."""
    return np.array(
        [
            [np.cos(yaw), -np.sin(yaw), x],
            [np.sin(yaw), np.cos(yaw), y],
            [0.0, 0.0, 1.0],
        ]
    )


def yaw_from_quaternion(w: float, x: float, y: float, z: float) -> float:
    """Extract the rotation around the z-axis, dropping roll and pitch."""
    return quat2euler([w, x, y, z], axes="sxyz")[2]


def compute_map_to_odom(world_pose: Pose, odom_pose: Pose, flip_side: bool) -> tuple[float, float, float]:
    """
    Compute the planar map -> odom transform so that map -> base_link matches the ground truth.

    :param world_pose: True pose of the robot's base link in the simulation world frame
    :param odom_pose: Pose of the robot's base link in the odom frame
    :param flip_side: Whether the map frame is rotated by half a turn relative to the world frame
    :returns: Translation (x, y) and yaw of the odom frame in the map frame
    """
    o = world_pose.orientation
    map_to_base = planar_transform(
        world_pose.position.x, world_pose.position.y, yaw_from_quaternion(o.w, o.x, o.y, o.z)
    )
    if flip_side:
        map_to_base = planar_transform(0.0, 0.0, np.pi) @ map_to_base

    o = odom_pose.orientation
    odom_to_base = planar_transform(odom_pose.position.x, odom_pose.position.y, yaw_from_quaternion(o.w, o.x, o.y, o.z))

    map_to_odom = map_to_base @ np.linalg.inv(odom_to_base)
    return map_to_odom[0, 2], map_to_odom[1, 2], np.arctan2(map_to_odom[1, 0], map_to_odom[0, 0])


class LocalizationFaker(Node):
    """
    Replaces the localization in simulation by publishing the map -> odom transform from the ground truth pose.

    The map frame has its origin in the center of the field and its x-axis points towards the opponent goal.
    The robot is assumed to start on its own half, so if it spawns on the positive x half of the simulation world,
    the map frame is rotated by half a turn relative to the world frame.
    """

    def __init__(self) -> None:
        super().__init__("localization_faker")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value

        self.flip_side: bool | None = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(PoseStamped, "ground_truth/pose", self.ground_truth_callback, 1)

    def ground_truth_callback(self, msg: PoseStamped) -> None:
        try:
            odom_to_base = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(str(ex), throttle_duration_sec=5.0)
            return

        # Decide once on which half the robot started, so the map frame stays fixed afterwards
        if self.flip_side is None:
            self.flip_side = msg.pose.position.x > 0
            self.get_logger().info(f"Robot started on the {'positive' if self.flip_side else 'negative'} x half")

        odom_pose = Pose()
        odom_pose.position.x = odom_to_base.transform.translation.x
        odom_pose.position.y = odom_to_base.transform.translation.y
        odom_pose.orientation = odom_to_base.transform.rotation
        x, y, yaw = compute_map_to_odom(msg.pose, odom_pose, self.flip_side)

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        q = euler2quat(0.0, 0.0, yaw, axes="sxyz")
        transform.transform.rotation.w = float(q[0])
        transform.transform.rotation.x = float(q[1])
        transform.transform.rotation.y = float(q[2])
        transform.transform.rotation.z = float(q[3])
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationFaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
