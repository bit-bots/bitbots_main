"""ROS node that localizes the robot from AprilTag detections.

AprilTags (and rigid tag bundles) are placed at known poses in the
environment, defined in a tag map YAML (see ``config/tag_map.yaml``). Every
camera image is searched for those tags; each (partially) visible bundle
yields a full 6D camera pose via a joint PnP over all of its detected tag
corners. The bundle poses are fused into the robot pose in the map frame,
published as a ``PoseWithCovarianceStamped`` and, optionally, as the
``map -> odom`` transform. That transform is re-broadcast continuously based
on the most recent detection, so tf consumers get a drift-free pose that
stays available and smooth while no tag is visible.
"""

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from bitbots_tf_buffer import Buffer
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from bitbots_apriltag_localization.detector import (
    AprilTagBundleDetector,
    BundleDetection,
    camera_pose_from_detections,
    draw_detections,
)
from bitbots_apriltag_localization.tag_map import load_tag_map


def _matrix_from_transform(transform) -> np.ndarray:
    """4x4 matrix from a geometry_msgs Transform."""
    rotation = transform.rotation
    translation = transform.translation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


class AprilTagLocalization(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_localization")

        # A relative tag map path is resolved inside this package's config directory.
        tag_map_path = Path(str(self.declare_parameter("tag_map", "tag_map.yaml").value))
        if not tag_map_path.is_absolute():
            tag_map_path = Path(get_package_share_directory("bitbots_apriltag_localization")) / "config" / tag_map_path
        tag_map = load_tag_map(tag_map_path)
        num_tags = sum(len(bundle.tags) for bundle in tag_map.bundles)
        self.get_logger().info(f"Loaded {len(tag_map.bundles)} bundles with {num_tags} tags from {tag_map_path}")

        self._detector = AprilTagBundleDetector(
            tag_map,
            nthreads=int(self.declare_parameter("detector.nthreads", 2).value),
            quad_decimate=float(self.declare_parameter("detector.quad_decimate", 2.0).value),
            quad_sigma=float(self.declare_parameter("detector.quad_sigma", 0.0).value),
            refine_edges=bool(self.declare_parameter("detector.refine_edges", True).value),
            decode_sharpening=float(self.declare_parameter("detector.decode_sharpening", 0.25).value),
        )

        self._map_frame = str(self.declare_parameter("frames.map", "map").value)
        self._odom_frame = str(self.declare_parameter("frames.odom", "odom").value)
        self._base_frame = str(self.declare_parameter("frames.base", "base_footprint").value)

        self._assume_rectified = bool(self.declare_parameter("camera.assume_rectified", True).value)
        self._max_reprojection_error = float(self.declare_parameter("max_reprojection_error", 5.0).value)
        self._publish_tf = bool(self.declare_parameter("publish_tf", True).value)
        self._transform_tolerance = Duration(seconds=float(self.declare_parameter("transform_tolerance", 0.2).value))
        tf_publish_rate = float(self.declare_parameter("tf_publish_rate", 20.0).value)

        position_variance = float(self.declare_parameter("position_variance", 0.05).value)
        orientation_variance = float(self.declare_parameter("orientation_variance", 0.02).value)
        self._covariance = np.diag([position_variance] * 3 + [orientation_variance] * 3).flatten()

        self._debug_detection_tf = bool(self.declare_parameter("debug.publish_detection_tf", False).value)
        debug_image = bool(self.declare_parameter("debug.publish_debug_image", False).value)

        self._tf_buffer = Buffer(node=self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._cv_bridge = CvBridge()

        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, "apriltag_pose", 1)
        self._debug_image_pub = self.create_publisher(Image, "apriltag_debug_image", 1) if debug_image else None

        # The last map -> odom correction. It stays valid while the robot only
        # accumulates odometry drift, so it is re-broadcast continuously and
        # detections merely update it.
        self._map_t_odom: Optional[np.ndarray] = None
        if self._publish_tf:
            self.create_timer(1.0 / tf_publish_rate, self._broadcast_map_to_odom)

        self.create_subscription(
            CameraInfo,
            str(self.declare_parameter("camera.camera_info_topic", "zed/zed_node/rgb/camera_info").value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.declare_parameter("camera.image_topic", "zed/zed_node/rgb/image_rect_color").value),
            self._image_callback,
            qos_profile_sensor_data,
        )

    # ------------------------------------------------------------------ #
    # callbacks
    # ------------------------------------------------------------------ #
    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._assume_rectified:
            # For a rectified image the projection matrix holds the intrinsics
            # and the distortion has already been compensated.
            self._detector.set_camera(np.array(msg.p).reshape(3, 4)[:, :3])
        else:
            self._detector.set_camera(np.array(msg.k).reshape(3, 3), np.array(msg.d) if len(msg.d) else None)

    def _image_callback(self, msg: Image) -> None:
        if not self._detector.has_camera:
            self.get_logger().warning("Waiting for camera info", throttle_duration_sec=5.0)
            return

        gray = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        detections = self._detector.detect(gray)
        good = [d for d in detections if d.reprojection_error <= self._max_reprojection_error]

        self._publish_debug(msg, gray, detections)
        if not good:
            return

        map_t_camera = camera_pose_from_detections(good)

        stamp = Time.from_msg(msg.header.stamp)
        camera_t_base = self._lookup_matrix(msg.header.frame_id, self._base_frame, stamp)
        if camera_t_base is None:
            return
        map_t_base = map_t_camera @ camera_t_base

        self._publish_pose(map_t_base, msg.header.stamp)
        if self._publish_tf:
            self._update_map_to_odom(map_t_base, stamp)

    # ------------------------------------------------------------------ #
    # outputs
    # ------------------------------------------------------------------ #
    def _publish_pose(self, map_t_base: np.ndarray, stamp) -> None:
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._map_frame
        position = pose.pose.pose.position
        position.x, position.y, position.z = map_t_base[:3, 3]
        orientation = pose.pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = Rotation.from_matrix(map_t_base[:3, :3]).as_quat()
        pose.pose.covariance = self._covariance
        self._pose_pub.publish(pose)

    def _update_map_to_odom(self, map_t_base: np.ndarray, stamp: Time) -> None:
        """Recomputes the map -> odom correction from a fresh detection."""
        odom_t_base = self._lookup_matrix(self._odom_frame, self._base_frame, stamp)
        if odom_t_base is None:
            return
        self._map_t_odom = map_t_base @ np.linalg.inv(odom_t_base)
        # Broadcast right away instead of waiting for the next timer tick.
        self._broadcast_map_to_odom()

    def _broadcast_map_to_odom(self) -> None:
        """Broadcasts the last known map -> odom correction."""
        if self._map_t_odom is None:
            return
        transform = TransformStamped()
        # Future-dated by the transform tolerance so the transform stays valid
        # between the periodic broadcasts.
        transform.header.stamp = (self.get_clock().now() + self._transform_tolerance).to_msg()
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        translation = transform.transform.translation
        translation.x, translation.y, translation.z = self._map_t_odom[:3, 3]
        rotation = transform.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = Rotation.from_matrix(self._map_t_odom[:3, :3]).as_quat()
        self._tf_broadcaster.sendTransform(transform)

    def _publish_debug(self, msg: Image, gray: np.ndarray, detections: list[BundleDetection]) -> None:
        if self._debug_detection_tf:
            for detection in detections:
                transform = TransformStamped()
                transform.header.stamp = msg.header.stamp
                transform.header.frame_id = msg.header.frame_id
                transform.child_frame_id = f"apriltag_{detection.bundle.name}"
                translation = transform.transform.translation
                translation.x, translation.y, translation.z = detection.camera_t_bundle[:3, 3]
                rotation = transform.transform.rotation
                rotation.x, rotation.y, rotation.z, rotation.w = Rotation.from_matrix(
                    detection.camera_t_bundle[:3, :3]
                ).as_quat()
                self._tf_broadcaster.sendTransform(transform)

        if self._debug_image_pub is not None and self._debug_image_pub.get_subscription_count() > 0:
            image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            draw_detections(image, detections, self._detector, self._max_reprojection_error)
            debug_msg = self._cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
            debug_msg.header = msg.header
            self._debug_image_pub.publish(debug_msg)

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #
    def _lookup_matrix(self, target_frame: str, source_frame: str, time: Time) -> Optional[np.ndarray]:
        """Pose of ``source_frame`` in ``target_frame`` as a 4x4 matrix, or None."""
        try:
            transform = self._tf_buffer.lookup_transform(target_frame, source_frame, time)
        except Exception:
            # The image stamp can be slightly outside the tf buffer; the latest
            # available transform is a good enough fallback.
            try:
                transform = self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
            except Exception as e:
                self.get_logger().warning(
                    f"Could not look up {source_frame} in {target_frame}: {e}", throttle_duration_sec=5.0
                )
                return None
        return _matrix_from_transform(transform.transform)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalization()

    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()


if __name__ == "__main__":
    main()
