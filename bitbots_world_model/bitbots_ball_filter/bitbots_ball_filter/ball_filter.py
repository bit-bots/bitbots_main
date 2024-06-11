#! /usr/bin/env python3
import math
from typing import Tuple, Union

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
    def __init__(self, position, header, confidence):
        self.position = position
        self.header = header
        self.confidence = confidence

    def get_header(self):
        return self.header

    def get_position(self):
        return self.position

    def get_confidence(self):
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
        self.config = self.param_listener.get_params()

        # creates kalmanfilter with 4 dimensions
        self.kf = KalmanFilter(dim_x=4, dim_z=2, dim_u=0)
        self.filter_initialized = False
        self.ball = None  # type: BallWrapper
        self.last_ball_stamp = None

        self.filter_time_step = 1.0 / self.config.filter_rate
        self.filter_reset_duration = Duration(seconds=self.config.filter_reset_time)
        self.logger.info(f"Using frame '{self.config.filter_frame}' for ball filtering")

        self.update_params()
        # adapt velocity factor to frequency
        self.velocity_factor = (1 - self.config.velocity_reduction) ** (self.filter_time_step)

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

        # setup subscriber
        self.subscriber = self.create_subscription(
            BallArray,
            self.config.ball_subscribe_topic,
            self.ball_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.reset_service = self.create_service(
            Trigger,
            self.config.ball_filter_reset_service_name,
            self.reset_filter_cb,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.filter_timer = self.create_timer(self.filter_time_step, self.filter_step)
        self.logger.info("Ball filter initialized")

    def reset_filter_cb(self, req, response) -> Tuple[bool, str]:
        self.logger.info("Resetting bitbots ball filter...")
        self.filter_initialized = False
        response.success = True
        return response

    def ball_callback(self, msg: BallArray) -> None:
        if msg.balls:  # Balls exist
            # Either select ball closest to previous prediction or with highest confidence
            if self.config.closest_distance_match:  # Select ball closest to previous prediction
                ball_msg = self._get_closest_ball_to_previous_prediction(msg)
            else:  # Select ball with highest confidence
                ball_msg = sorted(msg.balls, key=lambda ball: ball.confidence.confidence)[-1]

            # A ball measurement was selected, now we save it for the next filter step
            position = self._get_transform(msg.header, ball_msg.center)
            if position is not None:
                self.ball = BallWrapper(position, msg.header, ball_msg.confidence.confidence)

    def _get_closest_ball_to_previous_prediction(self, ball_array: BallArray) -> Union[Ball, None]:
        closest_distance = math.inf
        closest_ball_msg = ball_array.balls[0]
        for ball_msg in ball_array.balls:
            ball_transform = self._get_transform(ball_array.header, ball_msg.center)
            if ball_transform and self.ball:
                distance = math.dist(
                    (ball_transform.point.x, ball_transform.point.y),
                    (self.ball.get_position().point.x, self.ball.get_position().point.y),
                )
                if distance < closest_distance:
                    closest_ball_msg = ball_msg
        return closest_ball_msg

    def _get_transform(
        self, header: Header, point: Point, frame: Union[None, str] = None, timeout: float = 0.3
    ) -> Union[PointStamped, None]:
        if frame is None:
            frame = self.config.filter_frame

        point_stamped = PointStamped()
        point_stamped.header = header
        point_stamped.point = point

        try:
            return self.tf_buffer.transform(point_stamped, frame, timeout=Duration(nanoseconds=int(timeout * (10**9))))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.logger.warning(str(e))

    def update_params(self) -> None:
        """
        Updates parameters from dynamic reconfigure
        """
        self.config = self.param_listener.get_params()
        self.filter_reset_duration = Duration(seconds=self.config.filter_reset_time)
        self.velocity_factor = (1 - self.config.velocity_reduction) ** (self.filter_time_step)
        self.kf.Q = Q_discrete_white_noise(
            dim=2,
            dt=self.filter_time_step,
            var=self.config.process_noise_variance,
            block_size=2,
            order_by_dim=False,
        )

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

        if self.ball:  # Ball measurement exists
            # Reset filter, if distance between last prediction and latest measurement is too large
            distance_to_ball = math.dist(
                (self.kf.get_update()[0][0], self.kf.get_update()[0][1]), self.get_ball_measurement()
            )
            if self.filter_initialized and distance_to_ball > self.config.filter_reset_distance:
                self.filter_initialized = False
                self.logger.info(
                    f"Reset filter! Reason: Distance to ball {distance_to_ball} > {self.config.filter_reset_distance} (filter_reset_distance)"
                )
            # Initialize filter if not already
            if not self.filter_initialized:
                self.init_filter(*self.get_ball_measurement())
            # Predict and publish
            self.kf.predict()
            self.kf.update(self.get_ball_measurement())
            self.publish_data(*self.kf.get_update())
            self.last_ball_stamp = self.ball.get_header().stamp
            self.ball = None  # Clear handled measurement
        else:  # No new ball measurement to handle
            if self.filter_initialized:
                # Reset filer,if last measurement is too old
                age = self.get_clock().now() - rclpy.time.Time.from_msg(self.last_ball_stamp)
                if not self.last_ball_stamp or age > self.filter_reset_duration:
                    self.filter_initialized = False
                    self.logger.info(
                        f"Reset filter! Reason: Latest ball is too old {age} > {self.filter_reset_duration} (filter_reset_duration)"
                    )
                    return
                # Empty update, as no new measurement available (and not too old)
                self.kf.predict()
                self.kf.update(None)
                self.publish_data(*self.kf.get_update())
            else:  # Publish old state with huge covariance
                state_vec, cov_mat = self.kf.get_update()
                huge_cov_mat = np.eye(cov_mat.shape[0]) * 10
                self.publish_data(state_vec, huge_cov_mat)

    def get_ball_measurement(self) -> Tuple[float, float]:
        """extracts filter measurement from ball message"""
        return self.ball.get_position().point.x, self.ball.get_position().point.y

    def init_filter(self, x: float, y: float) -> None:
        """
        Initializes kalmanfilter at given position

        :param x: start x position of the ball
        :param y: start y position of the ball
        """
        self.logger.info(f"Initializing filter at position ({x}, {y})")
        # initial value of position(x,y) of the ball and velocity
        self.kf.x = np.array([x, y, 0, 0])

        # transition matrix
        self.kf.F = np.array(
            [
                [1.0, 0.0, 1.0, 0.0],
                [0.0, 1.0, 0.0, 1.0],
                [0.0, 0.0, self.velocity_factor, 0.0],
                [0.0, 0.0, 0.0, self.velocity_factor],
            ]
        )
        # measurement function
        self.kf.H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        # multiplying by the initial uncertainty
        self.kf.P = np.eye(4) * 1000

        # assigning measurement noise
        self.kf.R = np.array([[1, 0], [0, 1]]) * self.config.measurement_certainty

        # assigning process noise
        self.kf.Q = Q_discrete_white_noise(
            dim=2, dt=self.filter_time_step, var=self.config.process_noise_variance, block_size=2, order_by_dim=False
        )

        self.filter_initialized = True

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
        point_msg = Point()
        point_msg.x = float(state_vec[0])
        point_msg.y = float(state_vec[1])

        pos_covariance = np.eye(6).reshape(36)
        pos_covariance[0] = float(cov_mat[0][0])
        pos_covariance[1] = float(cov_mat[0][1])
        pos_covariance[6] = float(cov_mat[1][0])
        pos_covariance[7] = float(cov_mat[1][1])

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = header
        pose_msg.pose.pose.position = point_msg
        pose_msg.pose.covariance = pos_covariance
        pose_msg.pose.pose.orientation.w = 1.0
        self.ball_pose_publisher.publish(pose_msg)

        # velocity
        movement_msg = TwistWithCovarianceStamped()
        movement_msg.header = header
        movement_msg.twist.twist.linear.x = float(state_vec[2] * self.config.filter_rate)
        movement_msg.twist.twist.linear.y = float(state_vec[3] * self.config.filter_rate)
        movement_msg.twist.covariance = np.eye(6).reshape(36)
        movement_msg.twist.covariance[0] = float(cov_mat[2][2])
        movement_msg.twist.covariance[1] = float(cov_mat[2][3])
        movement_msg.twist.covariance[6] = float(cov_mat[3][2])
        movement_msg.twist.covariance[7] = float(cov_mat[3][3])
        self.ball_movement_publisher.publish(movement_msg)

        # ball
        ball_msg = PoseWithCertaintyStamped()
        ball_msg.header = header
        ball_msg.pose.pose.pose.position = point_msg
        ball_msg.pose.pose.covariance = pos_covariance
        ball_msg.pose.confidence = float(self.ball.get_confidence() if self.ball else 0.0)
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
