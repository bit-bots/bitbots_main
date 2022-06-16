#! /usr/bin/env python3
from typing import Union, Tuple

import math
import numpy as np

import rclpy
import tf2_ros as tf2

from copy import deepcopy
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter

from rclpy.node import Node
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from tf2_geometry_msgs import PointStamped
from humanoid_league_msgs.msg import PoseWithCertainty, PoseWithCertaintyArray, PoseWithCertaintyStamped


class BallFilter(Node):
    def __init__(self) -> None:
        """
        creates Kalmanfilter and subscribes to messages which are needed
        """
        super().__init__("ball_filter", automatically_declare_parameters_from_overrides=True)
        self.logger = self.get_logger()
        self.tf_buffer = tf2.Buffer(cache_time=rclpy.duration.Duration(seconds=2))
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self)
        # Setup dynamic reconfigure config
        self.config = {}
        self.add_on_set_parameters_callback(self._dynamic_reconfigure_callback)
        self._dynamic_reconfigure_callback(self.get_parameters_by_prefix("").values())

    def _dynamic_reconfigure_callback(self, config) -> SetParametersResult:
        tmp_config = deepcopy(self.config)
        for param in config:
            tmp_config[param.name] = param.value
        config = tmp_config
        # creates kalmanfilter with 4 dimensions
        self.kf = KalmanFilter(dim_x=4, dim_z=2, dim_u=0)
        self.filter_initialized = False
        self.ball = None  # type: PointStamped
        self.ball_header = None  # type: Header
        self.ball_msg = PoseWithCertainty()  # type: PoseWithCertainty

        self.filter_rate = config['filter_rate']
        self.min_ball_confidence = config['min_ball_confidence']
        self.measurement_certainty = config['measurement_certainty']
        self.filter_time_step = 1.0 / self.filter_rate
        self.filter_reset_duration = rclpy.duration.Duration(seconds=config['filter_reset_time'])
        self.filter_reset_distance = config['filter_reset_distance']
        self.closest_distance_match = config['closest_distance_match']

        filter_frame = config['filter_frame']
        if filter_frame == "odom":
            self.filter_frame = config['odom_frame']
        elif filter_frame == "map":
            self.filter_frame = config['map_frame']
        self.logger.info(f"Using frame '{self.filter_frame}' for ball filtering")

        # adapt velocity factor to frequency
        self.velocity_factor = (1 - config['velocity_reduction']) ** (1 / self.filter_rate)

        self.process_noise_variance = config['process_noise_variance']

        # publishes positions of ball
        self.ball_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            config['ball_position_publish_topic'],
            1
        )

        # publishes velocity of ball
        self.ball_movement_publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            config['ball_movement_publish_topic'],
            1
        )

        # publishes ball
        self.ball_publisher = self.create_publisher(
            PoseWithCertaintyStamped,
            config['ball_publish_topic'],
            1
        )

        # setup subscriber
        self.subscriber = self.create_subscription(
            PoseWithCertaintyArray,
            config['ball_subscribe_topic'],
            self.ball_callback,
            1
        )

        self.reset_service = self.create_service(
            Trigger,
            config['ball_filter_reset_service_name'],
            self.reset_filter_cb
        )

        self.config = config
        self.filter_timer = self.create_timer(self.filter_time_step, self.filter_step)
        return SetParametersResult(successful=True)

    def reset_filter_cb(self, req, response) -> Tuple[bool, str]:
        self.logger.info("Resetting bitbots ball filter...")
        self.filter_initialized = False
        return True, ""

    def ball_callback(self, msg: PoseWithCertaintyArray) -> None:
        if msg.poses:  # Balls exist
            # Either select ball closest to previous prediction or with highest confidence
            if self.closest_distance_match:  # Select ball closest to previous prediction
                self.ball_msg = self._get_closest_ball_to_previous_prediction(msg)
            else:  # Select ball with highest confidence
                self.ball_msg = sorted(msg.poses, key=lambda ball: ball.confidence)[-1]

            if self.ball_msg.confidence >= self.min_ball_confidence:
                self.ball = self._get_transform(msg.header, self.ball_msg.pose.pose.position)
                if self.ball:
                    self.ball_header = msg.header
                else:
                    self.ball_header = None

    def _get_closest_ball_to_previous_prediction(self, msg: PoseWithCertaintyArray) -> Union[PoseWithCertainty, None]:
        closest_distance = math.inf
        closest_ball_msg = None
        for ball_msg in msg.poses:
            ball_transform = self._get_transform(msg.header, ball_msg.pose.pose.position)
            if ball_transform:
                distance = math.dist(
                    (ball_transform.point.x, ball_transform.point.y),
                    (self.ball.point.x, self.ball.point.y))
                if distance < closest_distance:
                    closest_ball_msg  = ball_msg
        return closest_ball_msg

    def _get_transform(self,
            header: Header,
            point: Point,
            frame: Union[None, str] = None,
            timeout: float = 0.3) -> Union[PointStamped, None]:
        if frame is None:
            frame = self.filter_frame

        point_stamped = PointStamped()
        point_stamped.header = header
        point_stamped.point = point
        try:
            return self.tf_buffer.transform(point_stamped, frame, timeout=rclpy.duration.Duration(seconds=timeout))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.logger.warning(str(e))

    def filter_step(self) -> None:
        """"
        When ball has been assigned a value and filter has been initialized:
        State will be updated according to filter.
        If filter has not been initialized, then that will be done first.

        If there is no data for ball, a prediction will still be made and published
        Process noise is taken into account
        """
        if self.ball:  # Ball measurement exists
            distance_to_ball = math.dist(
                (self.kf.get_update()[0][0], self.kf.get_update()[0][1]),
                (self.ball.point.x, self.ball.point.y))
            if self.filter_initialized and distance_to_ball > self.filter_reset_distance:
                self.filter_initialized = False
            if not self.filter_initialized:
                self.init_filter(*self.get_ball_measurement())
            self.kf.predict()
            self.kf.update(self.get_ball_measurement())
            self.publish_data(*self.kf.get_update())
            self.ball = None
        else:
            if self.filter_initialized:
                # If last measurement is too old, reset filter
                if not self.ball_header or ((self.get_clock().now() - rclpy.time.Time.from_msg(self.ball_header.stamp)) > self.filter_reset_duration):
                    self.filter_initialized = False
                    return
                self.kf.predict()
                self.kf.update(None)
                self.publish_data(*self.kf.get_update())
            else:
                # Publish old state with huge covariance
                state_vec, cov_mat = self.kf.get_update()
                huge_cov_mat = np.eye(cov_mat.shape[0]) * 10
                self.publish_data(state_vec, huge_cov_mat)

    def get_ball_measurement(self) -> Tuple[float, float]:
        """extracts filter measurement from ball message"""
        try:
            return (self.ball.point.x, self.ball.point.y)
        except AttributeError as e:
            self.logger.warning(f"Did you reconfigure? Something went wrong... {e}")
            # TODO: Handle return values...

    def init_filter(self, x: float, y: float) -> None:
        """
        Initializes kalmanfilter at given position

        :param x: start x position of the ball
        :param y: start y position of the ball
        """
        # initial value of position(x,y) of the ball and velocity
        self.kf.x = np.array([x, y, 0, 0])

        # transition matrix
        self.kf.F = np.array([[1.0, 0.0, 1.0, 0.0],
                              [0.0, 1.0, 0.0, 1.0],
                              [0.0, 0.0, self.velocity_factor, 0.0],
                              [0.0, 0.0, 0.0, self.velocity_factor]])
        # measurement function
        self.kf.H = np.array([[1.0, 0.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0, 0.0]])
        # multiplying by the initial uncertainty
        self.kf.P = np.eye(4) * 1000

        # assigning measurement noise
        self.kf.R = np.array([[1, 0],
                              [0, 1]]) * self.measurement_certainty

        # assigning process noise
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=self.filter_time_step, var=self.process_noise_variance,
                                           block_size=2, order_by_dim=False)

        self.filter_initialized = True

    def publish_data(self, state_vec: np.array, cov_mat: np.array) -> None:
        """
        Publishes ball position and velocity to ros nodes
        :param state_vec: current state of kalmanfilter
        :param cov_mat: current covariance matrix
        """
        header = Header
        header.frame_id = self.filter_frame
        header.stamp = rclpy.time.Time.to_msg(self.get_clock().now())

        # position
        point_msg = Point()
        point_msg.x = float(state_vec[0])
        point_msg.y = float(state_vec[1])

        pos_covariance = np.eye(6).reshape((36))
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
        movement_msg.twist.twist.linear.x = float(state_vec[2] * self.filter_rate)
        movement_msg.twist.twist.linear.y = float(state_vec[3] * self.filter_rate)
        movement_msg.twist.covariance = np.eye(6).reshape((36))
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
        ball_msg.pose.confidence = self.ball_msg.confidence
        self.ball_publisher.publish(ball_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
