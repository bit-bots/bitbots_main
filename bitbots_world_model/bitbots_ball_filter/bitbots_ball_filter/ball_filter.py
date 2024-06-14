#! /usr/bin/env python3
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
import tf2_ros as tf2
from bitbots_tf_buffer import Buffer
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from soccer_vision_3d_msgs.msg import Ball, BallArray
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_geometry_msgs import PointStamped

from bitbots_ball_filter.ball_filter_parameters import bitbots_ball_filter as parameters
from bitbots_msgs.msg import PoseWithCertaintyStamped


class BallWrapper:
    def __init__(self, position: Point, header: Header, confidence: float):
        self.position = position
        self.header = header
        self.confidence = confidence
        self.processed = False

    def flag_processed(self) -> None:
        self.processed = True

    def get_processed(self) -> bool:
        return self.processed

    def get_header(self) -> Header:
        return self.header

    def get_position(self) -> Point:
        return self.position

    def get_position_tuple(self) -> Tuple[float, float]:
        return self.position.x, self.position.y

    def get_confidence(self) -> float:
        return self.confidence


class BallFilter(Node):
    def __init__(self) -> None:
        """
        creates Kalmanfilter and subscribes to messages which are needed
        """
        super().__init__("ball_filter")
        self.logger = self.get_logger()
        self.tf_buffer = Buffer(self, Duration(seconds=2))
        # Setup dynamic reconfigure config
        self.param_listener = parameters.ParamListener(self)

        self.kf: Optional[KalmanFilter] = None
        self.last_ball_measurement: Optional[BallWrapper] = None
        self.reset_requested = False

        self.update_params()

        self.logger.info(f"Using frame '{self.config.filter_frame}' for ball filtering")

        # publishes positions of ball
        self.ball_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.config.ball_position_publish_topic, 1
        )

        # publishes velocity of ball
        self.ball_movement_publisher = self.create_publisher(
            TwistWithCovarianceStamped, self.config.ball_movement_publish_topic, 1
        )

        # publishes ball
        self.ball_publisher = self.create_publisher(PoseWithCertaintyStamped, self.config.ball_publish_topic, 1)

        # Create callback group
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # setup subscriber
        self.subscriber = self.create_subscription(
            BallArray,
            self.config.ball_subscribe_topic,
            self.ball_callback,
            2,
            callback_group=self.callback_group,
        )

        self.reset_service = self.create_service(
            Trigger,
            self.config.ball_filter_reset_service_name,
            self.reset_filter_cb,
            callback_group=self.callback_group,
        )

        self.filter_timer = self.create_timer(
            self.filter_time_step, self.filter_step, callback_group=self.callback_group
        )
        self.logger.info("Ball filter initialized")

    def reset_filter_cb(self, req, response) -> Tuple[bool, str]:
        self.logger.info("Resetting bitbots ball filter...")
        # Reset filter
        self.reset_requested = True
        response.success = True
        return response

    def ball_callback(self, msg: BallArray) -> None:
        if msg.balls:  # Balls exist
            # If we have a kalman filter, we use it's estimate to ignore balls that are too far away (false positives)
            # The ignore distance is calculated using the filter's covariance and a factor
            # This way false positives are ignored if we already have a good estimate
            if self.kf is not None:
                ignore_threshold_x = math.sqrt(self.kf.P[0, 0]) * self.config.ignore_measurement_threshold
                ignore_threshold_y = math.sqrt(self.kf.P[0, 0]) * self.config.ignore_measurement_threshold

                # Filter out balls that are too far away from the filter's estimate
                filtered_balls: list[
                    tuple[Ball, PointStamped, float]
                ] = []  # Store, original ball in base_footprint frame, transformed ball in filter frame , distance to filter estimate
                ball: Ball
                for ball in msg.balls:
                    ball_transform = self._get_transform(msg.header, ball.center)
                    if ball_transform:
                        distance_x = ball_transform.point.x - self.kf.x[0]
                        distance_y = ball_transform.point.y - self.kf.x[1]
                        if abs(distance_x) < ignore_threshold_x and abs(distance_y) < ignore_threshold_y:
                            filtered_balls.append((ball, ball_transform, math.hypot(distance_x, distance_y)))
            else:
                filtered_balls = [(ball, self._get_transform(msg.header, ball.center), 0) for ball in msg.balls]

            # Select the ball with closest distance to the filter estimate
            # Return if no ball was found
            ball_msg, ball_measurement_map, _ = min(filtered_balls, key=lambda x: x[2], default=(None, None, 0))
            if ball_measurement_map is None:
                return

            # Store the ball measurement
            self.last_ball_measurement = BallWrapper(
                ball_measurement_map.point, ball_measurement_map.header, ball_msg.confidence.confidence
            )

            # Initialize filter if not already done
            # We do this here, because we need the ball measurement to initialize the filter
            if self.kf is None:
                self.init_filter(*self.last_ball_measurement.get_position_tuple())

            # Calculate distance from the robot to the ball and update measurement noise
            assert msg.header.frame_id == "base_footprint", "Ball frame_id is not 'base_footprint'!"
            robot_ball_delta = math.hypot(ball_msg.center.x, ball_msg.center.y)
            self.update_measurement_noise(robot_ball_delta)

    def _get_closest_ball_to_previous_prediction(self, ball_array: BallArray) -> Optional[Ball]:
        closest_distance = math.inf
        closest_ball_msg = ball_array.balls[0]
        for ball_msg in ball_array.balls:
            ball_transform = self._get_transform(ball_array.header, ball_msg.center)
            if ball_transform and self.last_ball_measurement:
                distance = math.dist(
                    (ball_transform.point.x, ball_transform.point.y),
                    self.last_ball_measurement.get_position_tuple(),
                )
                if distance < closest_distance:
                    closest_ball_msg = ball_msg
        return closest_ball_msg

    def _get_transform(
        self, header: Header, point: Point, frame: Optional[str] = None, timeout: float = 0.3
    ) -> Optional[PointStamped]:
        """
        Transforms a point to the filter frame
        """

        if frame is None:
            frame = self.config.filter_frame

        point_stamped = PointStamped()
        point_stamped.header = header
        point_stamped.point = point

        try:
            return self.tf_buffer.transform(point_stamped, frame, timeout=Duration(nanoseconds=int(timeout * (10**9))))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.logger.warning(str(e))

    def update_measurement_noise(self, distance: float) -> None:
        self.kf.R = np.eye(2) * (self.config.measurement_certainty + self.config.noise_increment_factor * distance**2)

    def update_params(self) -> None:
        """
        Updates parameters from dynamic reconfigure
        """
        self.config = self.param_listener.get_params()
        self.filter_time_step = 1.0 / self.config.filter_rate
        self.filter_reset_duration = Duration(seconds=self.config.filter_reset_time)
        if self.kf:
            self.setup_filter()

    def filter_step(self) -> None:
        """
        When ball has been assigned a value and filter has been initialized:
        State will be updated according to filter.
        If filter has not been initialized, then that will be done first.

        If there is no data for ball, a prediction will still be made and published
        Process noise is taken into account
        """
        # check whether parameters have changed
        if self.param_listener.is_old(self.config):
            self.param_listener.refresh_dynamic_parameters()
            self.update_params()

        # Reset filter if requested
        if self.reset_requested:
            self.kf = None
            self.reset_requested = False
            self.logger.info("Filter reset")

        # Early exit if filter is not initialized
        if self.kf is None:
            huge_cov_mat = np.eye(4) * 100
            self.publish_data(np.zeros((4,)), huge_cov_mat)
            return

        # Reset filer,if last measurement is too old
        age = self.get_clock().now() - rclpy.time.Time.from_msg(self.last_ball_measurement.get_header().stamp)
        if age > self.filter_reset_duration:
            self.kf = None
            self.logger.info(
                f"Reset filter! Reason: Latest ball is too old {age} > {self.filter_reset_duration} (filter_reset_duration)"
            )
            return

        # Predict next state
        self.kf.predict()
        # Update filter with new measurement if available
        if not self.last_ball_measurement.get_processed():
            self.last_ball_measurement.flag_processed()
            self.kf.update(self.last_ball_measurement.get_position_tuple())
        else:
            self.kf.update(None)
        self.publish_data(*self.kf.get_update())

    def init_filter(self, x: float, y: float) -> None:
        """
        Initializes kalman filter at given position

        :param x: start x position of the ball
        :param y: start y position of the ball
        """
        self.logger.info(f"Initializing filter at position ({x}, {y})")

        # Create Kalman filter
        self.kf = KalmanFilter(dim_x=4, dim_z=2, dim_u=0)

        # initial value of position(x,y) of the ball and velocity
        self.kf.x = np.array([x, y, 0, 0])

        # multiplying by the initial uncertainty
        self.kf.P = np.eye(4) * 1000

        # setup the other matrices, that can also be updated without reinitializing the filter
        self.setup_filter()

    def setup_filter(self) -> None:
        """
        Sets up the kalman filter with
        the different matrices
        """
        # Models the friction as an exponential decay alpha from the time constant tau (velocity_decay_time)
        # It is defined in a time-step independent way
        exponent_in_s = -self.filter_time_step / self.config.velocity_decay_time
        velocity_factor = 1 - math.exp(exponent_in_s)

        # transition matrix
        self.kf.F = np.array(
            [
                [1.0, 0.0, 1.0, 0.0],
                [0.0, 1.0, 0.0, 1.0],
                [0.0, 0.0, 1 - velocity_factor, 0.0],
                [0.0, 0.0, 0.0, 1 - velocity_factor],
            ]
        )
        # measurement function
        self.kf.H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])

        # assigning measurement noise
        self.kf.R = np.eye(2) * self.config.measurement_certainty

        # assigning process noise
        self.kf.Q = Q_discrete_white_noise(
            dim=2, dt=self.filter_time_step, var=self.config.process_noise_variance, block_size=2, order_by_dim=False
        )

    def publish_data(self, state_vec: np.array, cov_mat: np.array) -> None:
        """
        Publishes ball position and velocity to ros nodes
        :param state_vec: current state of kalmanfilter
        :param cov_mat: current covariance matrix
        """
        header = Header()
        header.frame_id = self.config.filter_frame
        header.stamp = rclpy.time.Time.to_msg(self.get_clock().now())

        # position
        point_msg = Point(x=float(state_vec[0]), y=float(state_vec[1]))

        # covariance
        pos_covariance = np.zeros((6, 6))
        pos_covariance[:2, :2] = cov_mat[:2, :2]

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = header
        pose_msg.pose.pose.position = point_msg
        pose_msg.pose.covariance = pos_covariance.reshape(-1)
        pose_msg.pose.pose.orientation.w = 1.0
        self.ball_pose_publisher.publish(pose_msg)

        # velocity
        movement_msg = TwistWithCovarianceStamped()
        movement_msg.header = header
        movement_msg.twist.twist.linear.x = float(state_vec[2] * self.config.filter_rate)
        movement_msg.twist.twist.linear.y = float(state_vec[3] * self.config.filter_rate)
        vel_covariance = np.eye(6)
        vel_covariance[:2, :2] = cov_mat[2:, 2:]
        movement_msg.twist.covariance = vel_covariance.reshape(-1)
        self.ball_movement_publisher.publish(movement_msg)

        # ball
        ball_msg = PoseWithCertaintyStamped()
        ball_msg.header = header
        ball_msg.pose.pose.pose.position = point_msg
        ball_msg.pose.pose.covariance = pos_covariance.reshape(-1)
        ball_msg.pose.confidence = float(
            self.last_ball_measurement.get_confidence() if self.last_ball_measurement else 0.0
        )
        self.ball_publisher.publish(ball_msg)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = BallFilter()
    # Number of executor threads is the number of MutiallyExclusiveCallbackGroups + 1 thread for the executor
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
