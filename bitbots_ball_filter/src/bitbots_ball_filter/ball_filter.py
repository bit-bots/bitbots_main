#! /usr/bin/env python3

import numpy as np
import rospy
import math
import tf2_ros as tf2
from dynamic_reconfigure.server import Server
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
from geometry_msgs.msg import (PoseWithCovarianceStamped,
                               TwistWithCovarianceStamped)
from humanoid_league_msgs.msg import (PoseWithCertainty,
                                      PoseWithCertaintyArray,
                                      PoseWithCertaintyStamped)
from std_msgs.msg import Header
from tf2_geometry_msgs import PointStamped

from bitbots_ball_filter.srv import ResetBallFilter
from bitbots_ball_filter.cfg import BallFilterConfig


class BallFilter:
    def __init__(self):
        """
        creates Kalmanfilter and subscribes to messages which are needed
        """
        rospy.init_node('ball_filter')

        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(2))
        self.tf_listener = tf2.TransformListener(self.tf_buffer)

        # Setup dynamic reconfigure config
        self.config = {}
        Server(BallFilterConfig, self._dynamic_reconfigure_callback)  # This also calls the callback once

        rospy.spin()

    def _dynamic_reconfigure_callback(self, config, level):
        """
        Callback for the dynamic reconfigure configuration.

        :param config: New _config
        :param level: The level is a definable int in the Vision.cfg file. All changed params are or ed together by dynamic reconfigure.
        """
        # creates kalmanfilter with 4 dimensions
        self.kf = KalmanFilter(dim_x=4, dim_z=2, dim_u=0)
        self.filter_initialized = False
        self.ball = None  # type: PointStamped
        self.ball_header = None  # type: Header
        self.last_ball_msg = None  # type: PoseWithCertainty

        self.filter_rate = config['filter_rate']
        self.measurement_certainty = config['measurement_certainty']
        self.filter_time_step = 1.0 / self.filter_rate
        self.filter_reset_duration = rospy.Duration(secs=config['filter_reset_time'])
        self.filter_reset_distance = config['filter_reset_distance']

        filter_frame = config.get('filter_frame')
        if filter_frame == "odom":
            self.filter_frame = rospy.get_param('~odom_frame')
        elif filter_frame == "map":
            self.filter_frame = rospy.get_param('~map_frame')
        rospy.loginfo(f"Using frame '{self.filter_frame}' for ball filtering", logger_name="ball_filter")

        # adapt velocity factor to frequency
        self.velocity_factor = (1 - config['velocity_reduction']) ** (1 / self.filter_rate)

        self.process_noise_variance = config['process_noise_variance']

        # publishes positions of ball
        self.ball_pose_publisher = rospy.Publisher(
            config['ball_position_publish_topic'],
            PoseWithCovarianceStamped,
            queue_size=1
        )

        # publishes velocity of ball
        self.ball_movement_publisher = rospy.Publisher(
            config['ball_movement_publish_topic'],
            TwistWithCovarianceStamped,
            queue_size=1
        )

        # publishes ball
        self.ball_publisher = rospy.Publisher(
            config['ball_publish_topic'],
            PoseWithCertaintyStamped,
            queue_size=1
        )

        # setup subscriber
        self.subscriber = rospy.Subscriber(
            config['ball_subscribe_topic'],
            PoseWithCertaintyArray,
            self.ball_callback,
            queue_size=1
        )
        
        self.reset_service = rospy.Service(
            config['ball_filter_reset_service_name'],
            ResetBallFilter,
            self.reset_filter_cb
        )

        self.config = config
        self.filter_timer = rospy.Timer(rospy.Duration(self.filter_time_step), self.filter_step)
        return config

    def ball_callback(self, msg: PoseWithCertaintyArray):
        """handles incoming ball messages"""
        if msg.poses:
            balls = sorted(msg.poses, reverse=True, key=lambda ball: ball.confidence)  # Sort all balls by confidence
            ball = balls[0]  # Ball with highest confidence

            if ball.confidence == 0:
                return
            self.last_ball_msg = ball
            ball_buffer = PointStamped(msg.header, ball.pose.pose.position)
            try:
                self.ball = self.tf_buffer.transform(ball_buffer, self.filter_frame, timeout=rospy.Duration(0.3))
                self.ball_header = msg.header
            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                rospy.logwarn(e)

    def reset_filter_cb(self, req):
        rospy.loginfo("Resetting bitbots ball filter...", logger_name="ball_filter")
        self.filter_initialized = False
        return True

    def filter_step(self, event):
        """"
        When ball has been assigned a value and filter has been initialized:
        state will be updated according to filter.
        If filter has not been initialized, then that will be done first.

        If there is not data for ball, a prediction will still be made and published
        Process noise is taken into account
        """
        if self.ball:
            if self.filter_initialized and self.distance_to_ball(self.kf.get_update()[0]) > self.filter_reset_distance:
                self.filter_initialized = False
            if not self.filter_initialized:
                self.init_filter(*self.get_ball_measurement())
            self.kf.predict()
            self.kf.update(self.get_ball_measurement())
            state = self.kf.get_update()
            self.publish_data(*state)
            self.last_state = state
            self.ball = None
        else: 
            if self.filter_initialized:
                if (rospy.Time.now() - self.ball_header.stamp) > self.filter_reset_duration:
                    self.filter_initialized = False
                    self.last_state = None
                    return
                self.kf.predict()
                self.kf.update(None)
                state = self.kf.get_update()
                self.publish_data(*state)
                self.last_state = state

    def distance_to_ball(self, state):
        state_x = state[0]
        state_y = state[1]
        ball_x = self.ball.point.x
        ball_y = self.ball.point.y
        # math.dist is implemented in Python 3.8
        return math.sqrt((state_x - ball_x) ** 2 + (state_y - ball_y) ** 2)

    def get_ball_measurement(self):
        """extracts filter measurement from ball message"""
        try:
            return np.array([self.ball.point.x, self.ball.point.y])
        except AttributeError as e:
            rospy.logwarn(f"Did you reconfigure? Something went wrong... {e}", logger_name="ball_filter")

    def init_filter(self, x, y):
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
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=self.filter_time_step, var=self.process_noise_variance, block_size=2, order_by_dim=False)

        self.filter_initialized = True

    def publish_data(self, state: np.array, cov_mat: np.array) -> None:
        """
        Publishes ball position and velocity to ros nodes
        :param state: current state of kalmanfilter
        :param cov_mat: current covariance matrix
        :return:
        """
        # position
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = self.filter_frame
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.pose.position.x = state[0]
        pose_msg.pose.pose.position.y = state[1]
        pose_msg.pose.covariance = np.eye(6).reshape((36))
        pose_msg.pose.covariance[0] = cov_mat[0][0]
        pose_msg.pose.covariance[1] = cov_mat[0][1]
        pose_msg.pose.covariance[6] = cov_mat[1][0]
        pose_msg.pose.covariance[7] = cov_mat[1][1]
        pose_msg.pose.pose.orientation.w = 1
        self.ball_pose_publisher.publish(pose_msg)
        # velocity
        movement_msg = TwistWithCovarianceStamped()
        movement_msg.header = pose_msg.header
        movement_msg.twist.twist.linear.x = state[2] * self.filter_rate
        movement_msg.twist.twist.linear.y = state[3] * self.filter_rate
        movement_msg.twist.covariance = np.eye(6).reshape((36))
        movement_msg.twist.covariance[0] = cov_mat[2][2]
        movement_msg.twist.covariance[1] = cov_mat[2][3]
        movement_msg.twist.covariance[6] = cov_mat[3][2]
        movement_msg.twist.covariance[7] = cov_mat[3][3]
        self.ball_movement_publisher.publish(movement_msg)
        # ball
        ball_msg = PoseWithCertaintyStamped()
        ball_msg.header = pose_msg.header
        ball_msg.pose.confidence = self.last_ball_msg.confidence
        ball_msg.pose.pose.pose.position = pose_msg.pose.pose.position
        ball_msg.pose.pose.covariance = pose_msg.pose.covariance
        self.ball_publisher.publish(ball_msg)


if __name__ == "__main__":
    BallFilter()
