#! /usr/bin/env python3
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
import tf2_ros as tf2
from bitbots_tf_buffer import Buffer
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from rclpy.time import Time
from ros2_numpy import msgify, numpify
from sensor_msgs.msg import CameraInfo
from soccer_vision_3d_msgs.msg import Ball, BallArray
from std_msgs.msg import Header, Int8
from std_srvs.srv import Trigger
from tf2_geometry_msgs import PointStamped, PoseStamped

from bitbots_ball_filter.ball_filter_parameters import bitbots_ball_filter as parameters
from bitbots_msgs.msg import TeamData

# Source id used for the observations this robot makes itself. Teammates are identified by
# their robot id, which is always larger than zero in the RoboCup protocol.
OWN_OBSERVATION_SOURCE = 0

# Source id published if the team ball is currently not backed by any observation
NO_OBSERVATION_SOURCE = -1


@dataclass
class BallObservation:
    """A ball observation made by a single robot of our team, expressed in the team ball frame."""

    position: np.ndarray
    covariance: np.ndarray
    # Distance between the observing robot and the ball it observed. Observations of nearby balls
    # are more trustworthy than observations of balls on the other side of the field.
    distance: float
    # Id of the robot that made the observation
    source: int
    # Time the observation was made
    stamp: Time


class AlphaBetaFilter:
    """
    Alpha-beta filter tracking the position and the velocity of a ball.

    The position is corrected towards every measurement with the alpha gain. The velocity is
    corrected with the beta gain, but only if the current and the previous measurement are
    consecutive observations of the same source. Two measurements of different sources are
    biased differently, so the difference between them describes the disagreement of the two
    observers and not the movement of the ball.

    If we did not receive a measurement for a while, we lost track of the ball. The next
    measurement then reinitializes the filter instead of correcting the outdated estimate.

    The uncertainty of the estimate grows over time and with negative observations, meaning
    situations in which we should have seen the ball but did not.
    """

    # Covariance of an estimate that is not backed by any measurement
    UNKNOWN_COVARIANCE = 1000.0

    def __init__(self) -> None:
        self._alpha: float = 0.0
        self._beta: float = 0.0
        self._max_measurement_gap: float = 0.0
        self.reset()

    def configure(self, alpha: float, beta: float, max_measurement_gap: float) -> None:
        self._alpha = alpha
        self._beta = beta
        self._max_measurement_gap = max_measurement_gap

    def reset(self) -> None:
        """Forgets the ball, so the next measurement reinitializes the filter"""
        self.position: np.ndarray = np.zeros(3)
        self.velocity: np.ndarray = np.zeros(3)
        self.covariance: np.ndarray = np.eye(3) * self.UNKNOWN_COVARIANCE
        self.covariance[2, 2] = 0.0  # Ignore z-axis
        self._last_measurement_stamp: Optional[Time] = None
        self._last_measurement_source: Optional[int] = None

    @property
    def source(self) -> Optional[int]:
        """Id of the robot whose observation the estimate is currently based on"""
        return self._last_measurement_source

    @property
    def last_measurement_stamp(self) -> Optional[Time]:
        """Time of the measurement the estimate is currently based on"""
        return self._last_measurement_stamp

    def is_newer_measurement(self, stamp: Time) -> bool:
        """Checks if a measurement carries new information or if we already used it"""
        return self._last_measurement_stamp is None or stamp > self._last_measurement_stamp

    def predict(self, time_step: float, process_noise: float) -> None:
        """Moves the estimate along the estimated velocity and increases its uncertainty"""
        self.position = self.position + self.velocity * time_step
        self.covariance[:2, :2] += np.eye(2) * process_noise
        self._limit_covariance()

    def update(self, position: np.ndarray, covariance: np.ndarray, source: int, stamp: Time) -> None:
        """Corrects the estimate with a new measurement"""
        gap = None if self._last_measurement_stamp is None else (stamp - self._last_measurement_stamp).nanoseconds / 1e9

        if gap is None or not 0.0 < gap <= self._max_measurement_gap:
            # We do not have a recent estimate that could be corrected, so we start over
            self.position = np.array(position)
            self.velocity = np.zeros(3)
            self.covariance = np.array(covariance)
        else:
            residual = position - self.position
            self.position = self.position + self._alpha * residual
            # Only the difference between two observations of the same source describes the
            # movement of the ball, so we skip the velocity estimation after a source switch
            if source == self._last_measurement_source:
                self.velocity = self.velocity + self._beta * residual / gap
                # The ball rolls on the field plane, so it has no vertical velocity
                self.velocity[2] = 0.0
            self.covariance = (1 - self._alpha) * self.covariance + self._alpha * covariance

        self._last_measurement_stamp = stamp
        self._last_measurement_source = source

    def add_negative_observation(self, factor: float) -> None:
        """Increases the uncertainty because we should have observed the ball but did not"""
        self.covariance = self.covariance * factor
        self._limit_covariance()

    def _limit_covariance(self) -> None:
        """Caps the uncertainty at the value of a completely unknown ball"""
        self.covariance = np.minimum(self.covariance, self.UNKNOWN_COVARIANCE)


class BallFilter(Node):
    config: parameters.Params

    def __init__(self) -> None:
        """
        creates the filters and subscribes to the messages which are needed
        """
        super().__init__("ball_filter")
        self.logger = self.get_logger()
        self.tf_buffer = Buffer(Duration(seconds=4), self)
        # Setup dynamic reconfigure config
        self.param_listener = parameters.ParamListener(self)

        # The ball as observed by this robot alone and the ball fused from all observations
        # made in our team, including the ones of this robot
        self.own_filter = AlphaBetaFilter()
        self.team_filter = AlphaBetaFilter()

        # Initialize parameters
        self.update_params()
        self.logger.info(
            f"Filtering our own ball in frame '{self.config.filter.frame}' "
            f"and the team ball in frame '{self.config.filter.team_frame}'"
        )
        self.camera_info: Optional[CameraInfo] = None

        # Initialize state
        self.team_data: dict[int, TeamData] = {}
        self.reset_balls()

        # publishes the position and the velocity of the ball observed by this robot
        self.ball_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.config.ros.ball_position_publish_topic, 1
        )
        self.ball_movement_publisher = self.create_publisher(
            TwistWithCovarianceStamped, self.config.ros.ball_movement_publish_topic, 1
        )

        # publishes the position and the velocity of the ball fused from all team observations
        self.team_ball_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.config.ros.team_ball_position_publish_topic, 1
        )
        self.team_ball_movement_publisher = self.create_publisher(
            TwistWithCovarianceStamped, self.config.ros.team_ball_movement_publish_topic, 1
        )
        self.team_ball_source_publisher = self.create_publisher(Int8, self.config.ros.team_ball_source_publish_topic, 1)

        # Create callback group
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # setup subscriber
        self.ball_subscriber = self.create_subscription(
            BallArray,
            self.config.ros.ball_subscribe_topic,
            self.ball_callback,
            2,
            callback_group=self.callback_group,
        )

        self.team_data_subscriber = self.create_subscription(
            TeamData,
            self.config.ros.team_data_subscribe_topic,
            self.team_data_callback,
            10,
            callback_group=self.callback_group,
        )

        self.camera_info_subscriber = self.create_subscription(
            CameraInfo, self.config.ros.camera_info_subscribe_topic, self.camera_info_callback, 1
        )

        self.reset_service = self.create_service(
            Trigger,
            self.config.ros.ball_filter_reset_service_name,
            self.reset_filter_cb,
            callback_group=self.callback_group,
        )

        self.filter_timer = self.create_timer(
            self.filter_time_step, self.filter_step, callback_group=self.callback_group
        )

    def reset_balls(self) -> None:
        """Forgets our own ball as well as the team ball"""
        self.own_filter.reset()
        self.team_filter.reset()
        # Drop the observations of our teammates too, as they would restore the team ball right away
        self.team_data.clear()

    def reset_filter_cb(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.logger.info("Resetting bitbots ball filter...")
        self.reset_balls()
        response.success = True
        return response

    def camera_info_callback(self, msg: CameraInfo):
        """Updates the camera intrinsics"""
        self.camera_info = msg

    def team_data_callback(self, msg: TeamData) -> None:
        """Stores the latest observation of a teammate"""
        self.team_data[msg.robot_id] = msg

    def ball_callback(self, msg: BallArray) -> None:
        """handles incoming ball detections form a frame captured by the camera"""

        # Keep track if we have updated the measurement
        # We might not have a measurement if the ball is not visible
        # We also filter out balls that are too far away from the filter's estimate
        ball_measurement_updated: bool = False

        # Do filtering, transform, ... if we have a ball
        if msg.balls:  # Balls exist in the frame
            # Ignore balls that are too far away (false positives)
            # The ignore distance is calculated using the filter's covariance and a factor
            # This way false positives are ignored if we already have a good estimate
            ignore_threshold_x, ignore_threshold_y, _ = (
                np.diag(self.own_filter.covariance) * self.config.filter.tracking.ignore_measurement_threshold
            )

            # Filter out balls that are too far away from the filter's estimate
            filtered_balls: list[
                tuple[Ball, PointStamped, float]
            ] = []  # Store, original ball in base_footprint frame, transformed ball in filter frame , distance to filter estimate
            ball: Ball
            for ball in msg.balls:
                # Bring ball detection into the frame of the filter and decouple the ball from the robots movement
                ball_transform = self._get_transform(msg.header, ball.center, self.config.filter.frame)
                # This checks if the ball can be transformed to the filter frame
                if ball_transform:
                    # Check if the ball is close enough to the filter's estimate
                    diff = numpify(ball_transform.point) - self.own_filter.position
                    if abs(diff[0]) < ignore_threshold_x and abs(diff[1]) < ignore_threshold_y:
                        # Store the ball relative to the robot, the ball in the filter frame and the distance to the filter estimate
                        filtered_balls.append((ball, ball_transform, float(np.linalg.norm(diff))))

            # Select the ball with closest distance to the filter estimate
            ball_msg, ball_measurement_map, _ = min(filtered_balls, key=lambda x: x[2], default=(None, None, 0))
            # Only update the measurement if we have a ball that is close enough to the filter's estimate
            if ball_measurement_map is not None and ball_msg is not None:
                # Estimate the covariance of the measurement based on the distance from the robot to the ball
                distance = float(np.linalg.norm(numpify(ball_msg.center)))
                self.own_filter.update(
                    numpify(ball_measurement_map.point),
                    self._measurement_covariance(distance),
                    OWN_OBSERVATION_SOURCE,
                    Time.from_msg(msg.header.stamp),
                )
                ball_measurement_updated = True

        # If we did not get a ball measurement, we can check if we should have seen the ball
        # And increase the covariance if we did not see the ball
        # Due to vision issues with very close balls we don't do this for close balls  # TODO Remove with 558
        if not ball_measurement_updated and self.is_estimate_in_fov(msg.header):
            self.own_filter.add_negative_observation(self.config.filter.covariance.negative_observation.value)

    def _measurement_covariance(self, distance: float) -> np.ndarray:
        """Calculates the covariance of a ball measurement based on the distance to the observer"""
        covariance = np.eye(3) * (
            self.config.filter.covariance.measurement_uncertainty
            + (distance**2) * self.config.filter.covariance.distance_factor
        )
        covariance[2, 2] = 0.0  # Ignore z-axis
        return covariance

    def _get_transform(self, header: Header, point: Point, frame: str, timeout: float = 0.3) -> Optional[PointStamped]:
        """
        Transforms a point into the given frame
        """
        point_stamped = PointStamped()
        point_stamped.header = header
        point_stamped.point = point

        try:
            return self.tf_buffer.transform(point_stamped, frame, timeout=Duration(nanoseconds=int(timeout * (10**9))))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.logger.warning(str(e), throttle_duration_sec=5.0)
            return None

    def update_params(self) -> None:
        """
        Updates parameters from dynamic reconfigure
        """
        self.config = self.param_listener.get_params()
        self.filter_time_step = 1.0 / self.config.filter.rate
        for ball_filter in (self.own_filter, self.team_filter):
            ball_filter.configure(
                self.config.filter.tracking.alpha,
                self.config.filter.tracking.beta,
                self.config.filter.tracking.max_measurement_gap,
            )

    def is_estimate_in_fov(self, header: Header) -> bool:
        """
        Calculates if a ball should be currently visible
        """
        # Check if we got a camera info to do this stuff
        if self.camera_info is None:
            self.logger.info("No camera info received. Not checking if the ball is currently visible.")
            return False

        # Build a pose
        ball_pose = PoseStamped()
        ball_pose.header.frame_id = self.config.filter.frame
        ball_pose.header.stamp = header.stamp
        ball_pose.pose.position = msgify(Point, self.own_filter.position)

        # Transform to camera frame
        try:
            ball_in_camera_optical_frame = self.tf_buffer.transform(
                ball_pose, self.camera_info.header.frame_id, timeout=Duration(nanoseconds=int(0.5 * 1e9))
            )
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.logger.warning(str(e))
            return False

        # Check if the ball is in front of the camera
        if ball_in_camera_optical_frame.pose.position.z < 0:
            return False

        # Quick math to get the pixel
        p = numpify(ball_in_camera_optical_frame.pose.position)
        k = np.reshape(self.camera_info.k, (3, 3))
        pixel = np.matmul(k, p)
        pixel = pixel * (1 / pixel[2])

        # Make sure that the transformed pixel is on the sensor and not too close to the border
        border_fraction = self.config.filter.covariance.negative_observation.ignore_border
        border_px = np.array([self.camera_info.width, self.camera_info.height]) / 2 * border_fraction
        in_fov_horizontal = bool(border_px[0] < pixel[0] <= self.camera_info.width - border_px[0])
        in_fov_vertical = bool(border_px[1] < pixel[1] <= self.camera_info.height - border_px[1])
        return in_fov_horizontal and in_fov_vertical

    def _own_observation(self) -> Optional[BallObservation]:
        """
        Provides the ball this robot observed itself as a candidate for the team ball.
        """
        # We can only contribute an observation if we know where the ball is
        stamp = self.own_filter.last_measurement_stamp
        if stamp is None or not self._is_certain_enough(self.own_filter.covariance):
            return None

        # The estimate of our own filter lives in a frame that moves with the robot, so we need to
        # bring it into the shared frame all robots of the team agree on
        header = Header(frame_id=self.config.filter.frame)
        position = self._get_transform(header, msgify(Point, self.own_filter.position), self.config.filter.team_frame)
        relative_position = self._get_transform(
            header, msgify(Point, self.own_filter.position), self.config.filter.base_footprint_frame
        )
        if position is None or relative_position is None:
            return None

        return BallObservation(
            position=numpify(position.point),
            # Our own covariance is isotropic, so it is not changed by the rotation between the frames
            covariance=self.own_filter.covariance,
            distance=float(np.linalg.norm(numpify(relative_position.point)[:2])),
            source=OWN_OBSERVATION_SOURCE,
            stamp=stamp,
        )

    def _teammate_observations(self, now: Time) -> list[BallObservation]:
        """
        Provides the balls our teammates observed as candidates for the team ball.
        """
        observations: list[BallObservation] = []
        data: TeamData
        for data in self.team_data.values():
            # Ignore teammates that are out of the game or that did not report for a while
            age = (now - Time.from_msg(data.header.stamp)).nanoseconds / 1e9
            if data.state == TeamData.STATE_PENALIZED or not 0.0 <= age <= self.config.filter.team.observation_timeout:
                continue

            if data.header.frame_id != self.config.filter.team_frame:
                self.logger.warning(
                    f"Ignoring team data of robot {data.robot_id} in frame '{data.header.frame_id}', "
                    f"as the team ball is filtered in frame '{self.config.filter.team_frame}'",
                    throttle_duration_sec=10.0,
                )
                continue

            # The teammate does not tell us how much its estimate degraded since it was sent,
            # so we age it ourselves with the same process noise we apply to our own estimate
            covariance = np.zeros((3, 3))
            reported_covariance = np.asarray(data.ball_absolute.covariance).reshape((6, 6))
            covariance[:2, :2] = reported_covariance[:2, :2] + np.eye(2) * (
                self.config.filter.covariance.process_noise * self.config.filter.rate * age
            )
            if not self._is_certain_enough(covariance):
                continue

            ball_position = numpify(data.ball_absolute.pose.position)
            observations.append(
                BallObservation(
                    position=ball_position,
                    covariance=covariance,
                    distance=float(np.linalg.norm(ball_position[:2] - numpify(data.robot_position.pose.position)[:2])),
                    source=data.robot_id,
                    stamp=Time.from_msg(data.header.stamp),
                )
            )
        return observations

    def _is_certain_enough(self, covariance: np.ndarray) -> bool:
        """Checks if an observation is precise enough to be used as a source for the team ball"""
        return bool(np.all(np.diag(covariance)[:2] < self.config.filter.team.max_covariance))

    def _select_team_observation(self, now: Time) -> Optional[BallObservation]:
        """
        Selects the observation the team ball should be based on.

        An observation is the better the more certain it is and the closer the observing robot is
        to the ball it reports. The source that is currently used is preferred, because switching
        between two similarly good sources would prevent any velocity estimation.
        """
        observations = self._teammate_observations(now)
        own_observation = self._own_observation()
        if own_observation is not None:
            observations.append(own_observation)

        def score(observation: BallObservation) -> float:
            switch_penalty = (
                0.0 if observation.source == self.team_filter.source else self.config.filter.team.source_switch_margin
            )
            return (
                self.config.filter.team.covariance_weight * float(np.max(np.diag(observation.covariance)[:2]))
                + self.config.filter.team.distance_weight * observation.distance
                + switch_penalty
            )

        return min(observations, key=score, default=None)

    def _publish_ball(
        self,
        ball_filter: AlphaBetaFilter,
        frame: str,
        stamp: Time,
        pose_publisher,
        movement_publisher,
    ) -> None:
        """Publishes the position and the velocity of the given filter"""
        header = Header(stamp=stamp.to_msg(), frame_id=frame)

        covariance = np.zeros((6, 6))
        covariance[:3, :3] = ball_filter.covariance

        pose_msg = PoseWithCovarianceStamped(header=header)
        pose_msg.pose.pose.position = msgify(Point, ball_filter.position)
        pose_msg.pose.pose.orientation.w = 1.0
        pose_msg.pose.covariance = covariance.flatten()
        pose_publisher.publish(pose_msg)

        movement_msg = TwistWithCovarianceStamped(header=header)
        movement_msg.twist.twist.linear.x = float(ball_filter.velocity[0])
        movement_msg.twist.twist.linear.y = float(ball_filter.velocity[1])
        movement_publisher.publish(movement_msg)

    def filter_step(self) -> None:
        """
        Steps both filters and publishes their estimates.

        Even without a new measurement a prediction is made and published,
        while the uncertainty of the estimate grows.
        """
        # check whether parameters have changed
        if self.param_listener.is_old(self.config):
            self.param_listener.refresh_dynamic_parameters()
            self.update_params()

        now = self.get_clock().now()
        process_noise = self.config.filter.covariance.process_noise

        # Step the ball we observed ourselves. It is corrected in the detection callback.
        self.own_filter.predict(self.filter_time_step, process_noise)
        self._publish_ball(
            self.own_filter,
            self.config.filter.frame,
            now,
            self.ball_pose_publisher,
            self.ball_movement_publisher,
        )

        # Step the team ball and correct it with the currently best observation of our team
        self.team_filter.predict(self.filter_time_step, process_noise)
        observation = self._select_team_observation(now)
        if observation is None:
            # Nobody in the team is able to tell us where the ball is
            self.team_filter.add_negative_observation(self.config.filter.covariance.negative_observation.value)
        elif self.team_filter.is_newer_measurement(observation.stamp):
            # Teammates report at a much lower rate than we step the filter, so we only correct
            # the estimate if the selected source actually provided something new
            self.team_filter.update(observation.position, observation.covariance, observation.source, observation.stamp)
        self._publish_ball(
            self.team_filter,
            self.config.filter.team_frame,
            now,
            self.team_ball_pose_publisher,
            self.team_ball_movement_publisher,
        )
        self.team_ball_source_publisher.publish(
            Int8(data=NO_OBSERVATION_SOURCE if observation is None else observation.source)
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = BallFilter()
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
