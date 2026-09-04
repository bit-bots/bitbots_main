import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import rclpy
import soccer_vision_2d_msgs.msg as sv2dm
import soccer_vision_3d_msgs.msg as sv3dm
import soccer_vision_attribute_msgs.msg as svam
import tf2_geometry_msgs
import tf2_ros as tf2
from bitbots_tf_buffer import Buffer
from geometry_msgs.msg import Point, Vector3
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from rclpy.time import Time
from ros2_numpy import numpify
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from bitbots_msgs.msg import TeamData

# Colors used to visualize the tracked robots depending on the team they belong to
TEAM_COLORS: dict[int, ColorRGBA] = {
    svam.Robot.TEAM_OWN: ColorRGBA(r=0.2, g=0.6, b=1.0, a=1.0),
    svam.Robot.TEAM_OPPONENT: ColorRGBA(r=1.0, g=0.3, b=0.2, a=1.0),
    svam.Robot.TEAM_UNKNOWN: ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0),
}


def angle_difference(angle: float, other: float) -> float:
    """Returns the signed difference between two angles in the range of -pi to pi"""
    return (angle - other + math.pi) % math.tau - math.pi


def radial_covariance(bearing: float, across_variance: float, along_variance: float) -> np.ndarray:
    """
    Builds a covariance that is oriented along the line of sight towards an object.

    We observe another robot by projecting its foot point in the image onto the field plane. The
    error of that projection is mostly radial: a few pixels of error move the foot point along the
    line of sight a lot, while the direction the robot is in stays accurate. The further away the
    robot is, the more pronounced this gets, which is why the two variances are not the same.
    """
    covariance = np.diag([along_variance, across_variance])
    rotation = np.array([[math.cos(bearing), -math.sin(bearing)], [math.sin(bearing), math.cos(bearing)]])
    return rotation @ covariance @ rotation.T


def directional_gain(gain: float, covariance: np.ndarray) -> np.ndarray:
    """
    Turns a filter gain into a gain matrix that respects how precise an observation is per direction.

    An observation is not equally precise in every direction, so weighting the gain with the
    observation covariance applies the correction where the observation actually carries
    information. For an observation that is equally precise in every direction this reduces to
    the plain gain.
    """
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    weights = np.min(eigenvalues) / eigenvalues
    return gain * (eigenvectors * weights) @ eigenvectors.T


@dataclass(frozen=True)
class TrackingConfig:
    """Gains and limits shared by all tracked robots"""

    # How much of the difference between observation and estimate is applied to the position
    alpha: float
    # How much of it is applied to the velocity
    beta: float
    # Two observations further apart than this are not consecutive, so we lost the velocity
    max_measurement_gap: float
    # Robots do not walk faster than this
    max_velocity: float


@dataclass(eq=False)
class TrackedRobot:
    """
    A robot on the field that we keep track of.

    Position and velocity are estimated with an alpha-beta filter on the field plane, corrected by
    the robot detections we were able to project onto the field. The velocity is only corrected
    between two consecutive detections, because otherwise the estimate moved for reasons we did
    not observe.

    The position is stored together with its covariance, so we know how certain we are about this
    particular robot. The covariance grows while we do not observe the robot and it shrinks with
    every detection, depending on how precise that detection is.
    """

    position: np.ndarray  # 2D position on the field plane in the filter frame
    covariance: np.ndarray  # 2x2 positional covariance
    size: Vector3  # Size of the bounding box as it was measured
    attributes: svam.Robot
    confidence: svam.Confidence
    stamp: Time  # Time we last observed this robot in any way
    measurement_stamp: Time  # Time we last observed where this robot is
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))
    # Covariance of the detection the estimate is based on. Observations that do not tell us where
    # the robot is must never make us more certain than this.
    measurement_covariance: np.ndarray = field(default_factory=lambda: np.eye(2))

    def predict(self, time_step: float, process_noise: float) -> None:
        """Moves the estimate along the estimated velocity and increases its uncertainty"""
        self.position = self.position + self.velocity * time_step
        self.covariance = self.covariance + np.eye(2) * process_noise

    def update(self, position: np.ndarray, covariance: np.ndarray, stamp: Time, config: TrackingConfig) -> None:
        """Corrects the estimate with a detection that tells us where the robot is"""
        gap = (stamp - self.measurement_stamp).nanoseconds / 1e9
        residual = position - self.position

        if 0.0 < gap <= config.max_measurement_gap:
            velocity = self.velocity + directional_gain(config.beta, covariance) @ residual / gap
            speed = float(np.linalg.norm(velocity))
            self.velocity = velocity if speed <= config.max_velocity else velocity / speed * config.max_velocity
        else:
            # We did not see this robot for a while, so whatever it did in the meantime is not
            # described by this detection
            self.velocity = np.zeros(2)

        gain = directional_gain(config.alpha, covariance)
        self.position = self.position + gain @ residual
        # Joseph form, which stays symmetric and positive for any gain we choose
        remainder = np.eye(2) - gain
        self.covariance = remainder @ self.covariance @ remainder.T + gain @ covariance @ gain.T

        self.measurement_covariance = covariance
        self.measurement_stamp = stamp
        self.stamp = stamp

    def add_positive_observation(self, factor: float, stamp: Time) -> None:
        """
        Slows down how fast we get uncertain about a robot we detected but could not locate.

        A detection whose foot point we can not project onto the field plane does not tell us
        where the robot is, so it must not move our estimate. It does tell us that the robot is
        still there, which is why we let our certainty decay slower than it would without any
        observation, but never to the point where we are as certain as an actual detection.
        """
        confirmed_covariance = self.covariance * factor
        if np.trace(confirmed_covariance) >= np.trace(self.measurement_covariance):
            self.covariance = confirmed_covariance
        self.stamp = stamp

    def add_negative_observation(self, factor: float) -> None:
        """Increases the uncertainty because we should have observed the robot but did not"""
        self.covariance = self.covariance * factor

    @property
    def uncertainty(self) -> float:
        """The largest variance of the estimate, used to decide if we still trust it"""
        return float(np.max(np.diag(self.covariance)))

    @property
    def height(self) -> float:
        return self.size.z

    def to_msg(self) -> sv3dm.Robot:
        robot = sv3dm.Robot()
        robot.bb.center.position = Point(x=float(self.position[0]), y=float(self.position[1]), z=self.height / 2)
        robot.bb.center.orientation.w = 1.0
        robot.bb.size = self.size
        robot.attributes = self.attributes
        robot.confidence = self.confidence
        return robot


class RobotFilter(Node):
    def __init__(self):
        super().__init__("bitbots_robot_filter")

        self.tf_buffer = Buffer(Duration(seconds=10.0), self)

        # All callbacks work on the same tracked robots, so they share one callback group
        self.callback_group = MutuallyExclusiveCallbackGroup()

        self.robots: list[TrackedRobot] = list()
        self.team: dict[int, TeamData] = dict()
        self.camera_info: Optional[CameraInfo] = None

        self.filter_frame = self.declare_parameter("filter_frame", "map").value
        self.base_footprint_frame = self.declare_parameter("base_footprint_frame", "base_footprint").value
        self.robot_dummy_size = self.declare_parameter("robot_dummy_size", 0.4).value
        self.robot_storage_time = self.declare_parameter("robot_storage_time", 10e9).value
        self.team_data_timeout = self.declare_parameter("team_data_timeout", 1e9).value

        # Uncertainty of a single detection. The error of the projection onto the field plane is
        # mostly radial, so it grows much faster along the line of sight than across it.
        self.measurement_uncertainty = self.declare_parameter("measurement_uncertainty", 0.05).value
        self.distance_factor = self.declare_parameter("distance_factor", 0.02).value
        self.heading_uncertainty = self.declare_parameter("heading_uncertainty", 0.05).value

        # Uncertainty that is added in every filter step, as the robots move while we do not look
        # at them. Robots that are further away suffer from a larger projection error, so our
        # estimate of them degrades faster.
        self.process_noise = self.declare_parameter("process_noise", 0.005).value
        self.process_noise_distance_factor = self.declare_parameter("process_noise_distance_factor", 0.0005).value

        # Estimates that got more uncertain than this are dropped
        self.max_covariance = self.declare_parameter("max_covariance", 1.0).value

        # Factor by which the uncertainty grows if we do not detect a robot we should have seen
        self.negative_observation_value = self.declare_parameter("negative_observation.value", 1.05).value
        # Border of the image in which we do not count a missing detection as a negative observation,
        # because robots are only partially visible there
        self.negative_observation_ignore_border = self.declare_parameter(
            "negative_observation.ignore_border", 0.05
        ).value
        # Factor by which the uncertainty shrinks if we detect a robot in the image but can not
        # tell where it is. It has to be close to one, so it only slows the decay of our certainty.
        self.positive_observation_value = self.declare_parameter("positive_observation.value", 0.98).value

        # A detection belongs to a robot we track if it is roughly in the same direction and
        # roughly at the same distance. The distance is the far less reliable of the two, so it
        # gets its own threshold which grows with the distance to the robot.
        self.matching_heading = self.declare_parameter("matching.heading", 0.25).value
        self.matching_range = self.declare_parameter("matching.range", 0.4).value
        self.matching_range_distance_factor = self.declare_parameter("matching.range_distance_factor", 0.15).value
        # Maximum angle between a robot we track and a detection in the image that we still
        # consider to be the same robot
        self.matching_image_heading = self.declare_parameter("matching.image_heading", 0.25).value

        self.filter_time_step = 1 / 20
        self.tracking_config = TrackingConfig(
            alpha=self.declare_parameter("tracking.alpha", 0.4).value,
            beta=self.declare_parameter("tracking.beta", 0.1).value,
            max_measurement_gap=self.declare_parameter("tracking.max_measurement_gap", 0.5).value,
            max_velocity=self.declare_parameter("tracking.max_velocity", 1.0).value,
        )

        self.create_subscription(
            sv3dm.RobotArray,
            self.declare_parameter("robot_observation_topic", "robots_relative").value,
            self._robot_vision_callback,
            5,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            sv2dm.RobotArray,
            self.declare_parameter("robot_image_observation_topic", "robots_in_image").value,
            self._robot_image_callback,
            5,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            CameraInfo,
            self.declare_parameter("camera_info_topic", "camera/camera_info").value,
            self._camera_info_callback,
            1,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            TeamData,
            self.declare_parameter("team_data_topic", "team_data").value,
            self._team_data_callback,
            5,
            callback_group=self.callback_group,
        )

        self.robot_obstacle_publisher = self.create_publisher(
            sv3dm.RobotArray, self.declare_parameter("robots_publish_topic", "robots_relative_filtered").value, 1
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray, self.declare_parameter("debug_marker_topic", "debug/robot_filter/markers").value, 1
        )

        self.create_timer(self.filter_time_step, self.publish_obstacles, callback_group=self.callback_group)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Updates the camera intrinsics"""
        self.camera_info = msg

    def _team_data_callback(self, msg: TeamData):
        self.team[msg.robot_id] = msg

    ###############
    # Observation #
    ###############

    def _measurement_covariance(self, bearing: float, distance: float) -> np.ndarray:
        """
        Calculates the covariance of a detection based on where it was observed.

        The error of the projection onto the field plane grows with the square of the distance
        along the line of sight, while the direction the robot was seen in stays accurate.
        """
        return radial_covariance(
            bearing,
            across_variance=self.measurement_uncertainty + (distance * self.heading_uncertainty) ** 2,
            along_variance=self.measurement_uncertainty + distance**2 * self.distance_factor,
        )

    def _matching_robot(self, position: np.ndarray, observer_position: np.ndarray) -> Optional[TrackedRobot]:
        """
        Returns the robot we track that a detection at the given position belongs to.

        A detection and an estimate belong to the same robot if we see them in roughly the same
        direction and at roughly the same distance. The distance we accept grows with the distance
        of the robot, because the projection of a far away robot onto the field plane is a lot
        less precise than the direction we saw it in.
        """
        bearing, distance = self._bearing_and_distance(position, observer_position)

        def is_same_robot(robot: TrackedRobot) -> bool:
            robot_bearing, robot_distance = self._bearing_and_distance(robot.position, observer_position)
            accepted_range = self.matching_range + robot_distance * self.matching_range_distance_factor
            return (
                abs(angle_difference(bearing, robot_bearing)) < self.matching_heading
                and abs(distance - robot_distance) < accepted_range
            )

        candidates = [robot for robot in self.robots if is_same_robot(robot)]
        return min(candidates, key=lambda robot: float(np.linalg.norm(position - robot.position)), default=None)

    @staticmethod
    def _bearing_and_distance(position: np.ndarray, observer_position: np.ndarray) -> tuple[float, float]:
        """Returns the direction and the distance a position was observed at"""
        offset = position - observer_position
        return math.atan2(offset[1], offset[0]), float(np.linalg.norm(offset))

    def _robot_vision_callback(self, msg: sv3dm.RobotArray):
        """
        Handles robot detections that the inverse perspective mapping was able to project
        onto the field plane, so they tell us where a robot is.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.filter_frame, msg.header.frame_id, msg.header.stamp, Duration(seconds=1.0)
            )
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return

        stamp = Time.from_msg(msg.header.stamp)
        # Where we stood while we observed the robots
        observer_position = numpify(transform.transform.translation)[:2]

        robot: sv3dm.Robot
        for robot in msg.robots:
            # Transform robot into the frame of the filter
            robot.bb.center = tf2_geometry_msgs.do_transform_pose(robot.bb.center, transform)
            position = numpify(robot.bb.center.position)[:2]
            bearing, distance = self._bearing_and_distance(position, observer_position)
            covariance = self._measurement_covariance(bearing, distance)

            tracked_robot = self._matching_robot(position, observer_position)
            if tracked_robot is None:
                self.robots.append(
                    TrackedRobot(
                        position=position,
                        covariance=covariance,
                        size=robot.bb.size,
                        attributes=robot.attributes,
                        confidence=robot.confidence,
                        stamp=stamp,
                        measurement_stamp=stamp,
                        measurement_covariance=covariance,
                    )
                )
            else:
                tracked_robot.update(position, covariance, stamp, self.tracking_config)
                tracked_robot.size = robot.bb.size
                tracked_robot.attributes = robot.attributes
                tracked_robot.confidence = robot.confidence

    def _robot_image_callback(self, msg: sv2dm.RobotArray):
        """
        Handles robot detections in the image.

        The inverse perspective mapping drops detections whose foot point it can not project onto
        the field plane, for example because the robot is cut off at the bottom of the image or
        because it is above the horizon. Those detections do not tell us where the robot is, so
        they do not move our estimate, but they do tell us that the robot is still there. We
        therefore let our certainty about it decay slower than it would without any observation.

        A robot we track that should be visible but is not detected at all is a negative
        observation, which makes us less certain about it until we drop it.
        """
        if self.camera_info is None:
            self.get_logger().info(
                "No camera info received. Not using the robot detections in the image.", throttle_duration_sec=10.0
            )
            return

        camera_frame = self.camera_info.header.frame_id
        try:
            camera_to_filter = numpify(
                self.tf_buffer.lookup_transform(
                    self.filter_frame, camera_frame, msg.header.stamp, Duration(seconds=1.0)
                ).transform
            )
            filter_to_camera = numpify(
                self.tf_buffer.lookup_transform(
                    camera_frame, self.filter_frame, msg.header.stamp, Duration(seconds=1.0)
                ).transform
            )
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return

        stamp = Time.from_msg(msg.header.stamp)
        camera_position = camera_to_filter[:2, 3]

        # Direction of every detection in the image, as an angle on the field plane
        detection_bearings = [
            self._bearing_of_detection(detection, camera_to_filter[:3, :3]) for detection in msg.robots
        ]

        for tracked_robot in self.robots:
            pixel = self._project_to_image(tracked_robot, filter_to_camera)
            # We can not tell anything about robots that are not in the image
            if pixel is None:
                continue

            bearing, _ = self._bearing_and_distance(tracked_robot.position, camera_position)
            closest_bearing = min(
                detection_bearings, key=lambda detection: abs(angle_difference(detection, bearing)), default=None
            )

            if closest_bearing is not None and abs(angle_difference(closest_bearing, bearing)) < (
                self.matching_image_heading
            ):
                # We see a robot in the direction we expect this one to be in
                tracked_robot.add_positive_observation(self.positive_observation_value, stamp)
            elif self._is_well_inside_image(pixel):
                # We looked right at it and did not see anything
                tracked_robot.add_negative_observation(self.negative_observation_value)

    def _bearing_of_detection(self, detection: sv2dm.Robot, camera_rotation: np.ndarray) -> float:
        """
        Calculates the direction a detection in the image is in as an angle on the field plane.
        """
        assert self.camera_info is not None, "The camera intrinsics are needed to calculate a bearing"
        # Ray through the center of the detection in the optical frame of the camera
        pixel = np.array([detection.bb.center.position.x, detection.bb.center.position.y, 1.0])
        ray = np.linalg.inv(np.reshape(self.camera_info.k, (3, 3))) @ pixel
        # Bring the ray into the frame of the filter and drop its vertical component
        ray = camera_rotation @ ray
        return math.atan2(ray[1], ray[0])

    def _project_to_image(self, robot: TrackedRobot, filter_to_camera: np.ndarray) -> Optional[np.ndarray]:
        """
        Projects the center of a tracked robot into the image and returns its pixel.

        Returns None if the robot is behind the camera or outside of the image.
        """
        assert self.camera_info is not None, "The camera intrinsics are needed to project into the image"
        # We project the center of the robot instead of its foot point, because a robot that is
        # close to us is cut off at the bottom of the image while its center is still visible
        position = np.array([robot.position[0], robot.position[1], robot.height / 2, 1.0])
        position_in_camera = (filter_to_camera @ position)[:3]

        # The robot is behind the camera
        if position_in_camera[2] <= 0:
            return None

        pixel = np.reshape(self.camera_info.k, (3, 3)) @ position_in_camera
        pixel = pixel[:2] / pixel[2]

        on_sensor = 0 <= pixel[0] < self.camera_info.width and 0 <= pixel[1] < self.camera_info.height
        return pixel if on_sensor else None

    def _is_well_inside_image(self, pixel: np.ndarray) -> bool:
        """
        Checks that a pixel is not close to the border of the image.

        We do not draw conclusions from a missing detection close to the border,
        as robots are only partially visible there.
        """
        assert self.camera_info is not None, "The camera intrinsics are needed to check the image border"
        border = (
            np.array([self.camera_info.width, self.camera_info.height]) / 2 * self.negative_observation_ignore_border
        )
        return bool(
            border[0] < pixel[0] <= self.camera_info.width - border[0]
            and border[1] < pixel[1] <= self.camera_info.height - border[1]
        )

    ##############
    # Publishing #
    ##############

    def _observer_position(self) -> Optional[np.ndarray]:
        """Returns where we currently stand in the frame of the filter"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.filter_frame, self.base_footprint_frame, Time(), Duration(seconds=0.2)
            )
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return None
        return numpify(transform.transform.translation)[:2]

    def _process_noise(self, robot: TrackedRobot, observer_position: Optional[np.ndarray]) -> float:
        """
        Returns how much uncertainty a robot gathers in one filter step.

        Robots that are further away gather it faster, because the error of projecting them onto
        the field plane grows with their distance, which means our estimate of them degrades as
        soon as we or they move a little.
        """
        if observer_position is None:
            return self.process_noise
        _, distance = self._bearing_and_distance(robot.position, observer_position)
        return self.process_noise + distance**2 * self.process_noise_distance_factor

    def publish_obstacles(self):
        # Set current timestamp and frame
        dummy_header = Header()
        dummy_header.stamp = self.get_clock().now().to_msg()
        dummy_header.frame_id = self.filter_frame

        # The robots keep moving while we do not observe them, so we follow their estimated
        # velocity and get less certain about them
        observer_position = self._observer_position()
        for tracked_robot in self.robots:
            tracked_robot.predict(self.filter_time_step, self._process_noise(tracked_robot, observer_position))

        # Drop the robots we are not certain about anymore or that we did not see for a long time
        def is_still_tracked(robot: TrackedRobot) -> bool:
            age = abs((self.get_clock().now() - robot.stamp).nanoseconds)
            return robot.uncertainty < self.max_covariance and age < self.robot_storage_time

        self.robots = list(filter(is_still_tracked, self.robots))

        # Convert TeamData to Robot Observation
        def build_robot_detection_from_team_data(msg: TeamData) -> sv3dm.Robot:
            robot = sv3dm.Robot()
            robot.bb.center = msg.robot_position.pose
            robot.bb.size.x = self.robot_dummy_size
            robot.bb.size.y = self.robot_dummy_size
            robot.bb.size.z = 1.0
            robot.attributes.team = svam.Robot.TEAM_OWN
            robot.attributes.player_number = msg.robot_id
            return robot

        def is_team_data_fresh(msg: TeamData) -> bool:
            return (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds < self.team_data_timeout

        # We want a new list, so the team data is only applied temporarily
        robots = [robot.to_msg() for robot in self.robots]

        # Add Team Mates (if the data is fresh enough)
        robots.extend(map(build_robot_detection_from_team_data, filter(is_team_data_fresh, self.team.values())))

        # Publish the robot obstacles
        self.robot_obstacle_publisher.publish(sv3dm.RobotArray(header=dummy_header, robots=robots))
        self.publish_markers(dummy_header)

    def publish_markers(self, header: Header) -> None:
        """
        Publishes a marker per tracked robot and an ellipse showing how certain we are about it.
        """
        # Start with a clean slate, so the markers of robots we dropped do not stay in the view
        markers = [Marker(header=header, action=Marker.DELETEALL)]

        for index, robot in enumerate(self.robots):
            color = TEAM_COLORS.get(robot.attributes.team, TEAM_COLORS[svam.Robot.TEAM_UNKNOWN])

            body = Marker(header=header, ns="robots", id=index, type=Marker.CYLINDER, action=Marker.ADD)
            body.pose.position = Point(x=float(robot.position[0]), y=float(robot.position[1]), z=robot.height / 2)
            body.pose.orientation.w = 1.0
            body.scale = Vector3(x=robot.size.x, y=robot.size.y, z=robot.height)
            # The more uncertain we are about a robot, the more transparent we draw it
            body.color = ColorRGBA(
                r=color.r, g=color.g, b=color.b, a=float(1.0 - robot.uncertainty / self.max_covariance) * 0.8 + 0.2
            )
            markers.append(body)

            markers.append(self._covariance_marker(header, index, robot, color))
            markers.append(self._velocity_marker(header, index, robot, color))

            label = Marker(header=header, ns="labels", id=index, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.pose.position = Point(x=float(robot.position[0]), y=float(robot.position[1]), z=robot.height + 0.15)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.2
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = (
                f"#{robot.attributes.player_number} "
                f"σ={math.sqrt(robot.uncertainty):.2f}m "
                f"v={float(np.linalg.norm(robot.velocity)):.2f}m/s"
            )
            markers.append(label)

        self.marker_publisher.publish(MarkerArray(markers=markers))

    def _velocity_marker(self, header: Header, index: int, robot: TrackedRobot, color: ColorRGBA) -> Marker:
        """
        Builds an arrow showing where a tracked robot is heading and how fast.
        """
        marker = Marker(header=header, ns="velocity", id=index, type=Marker.ARROW, action=Marker.ADD)
        start = Point(x=float(robot.position[0]), y=float(robot.position[1]), z=robot.height)
        # The arrow shows where the robot walks to within one second
        marker.points = [
            start,
            Point(x=start.x + float(robot.velocity[0]), y=start.y + float(robot.velocity[1]), z=start.z),
        ]
        # Shaft diameter, head diameter and head length
        marker.scale = Vector3(x=0.03, y=0.07, z=0.1)
        marker.color = color
        return marker

    def _covariance_marker(self, header: Header, index: int, robot: TrackedRobot, color: ColorRGBA) -> Marker:
        """
        Builds a flat ellipse on the ground showing the one sigma uncertainty of a tracked robot.
        """
        # The eigenvectors of the covariance point along the axes of the ellipse and the square
        # roots of the eigenvalues are the standard deviations along them
        eigenvalues, eigenvectors = np.linalg.eigh(robot.covariance)
        deviations = np.sqrt(np.maximum(eigenvalues, 0.0))
        # eigh returns the eigenvalues in ascending order, so the last vector is the long axis
        yaw = math.atan2(eigenvectors[1, -1], eigenvectors[0, -1])

        marker = Marker(header=header, ns="uncertainty", id=index, type=Marker.CYLINDER, action=Marker.ADD)
        marker.pose.position = Point(x=float(robot.position[0]), y=float(robot.position[1]), z=0.01)
        marker.pose.orientation.z = math.sin(yaw / 2)
        marker.pose.orientation.w = math.cos(yaw / 2)
        # The ellipse is drawn with its long axis along x, which is why the deviations are swapped
        marker.scale = Vector3(x=float(2 * deviations[-1]), y=float(2 * deviations[0]), z=0.01)
        marker.color = ColorRGBA(r=color.r, g=color.g, b=color.b, a=0.35)
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = RobotFilter()
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
