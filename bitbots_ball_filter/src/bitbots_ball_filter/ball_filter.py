#! /usr/bin/env python3

import rospy
import tf2_ros as tf2
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from tf2_geometry_msgs import PointStamped
from humanoid_league_msgs.msg import PoseWithCertaintyArray, PoseWithCertaintyStamped, PoseWithCertainty


class BallFilter:
    def __init__(self):
        """
        creates Kalmanfilter and subscribes to messages which are needed
        """
        rospy.init_node('ball_filter')
        #creates kalmanfilter with 4 dimensions
        self.kf = KalmanFilter(dim_x=4, dim_z=2, dim_u=0)
        self.filter_init = False
        self.ball = None  # type:PointStamped
        self.ball_header = None
        self.last_ball_msg = None  # type: PoseWithCertainty
        self.last_state = None

        # setup subscriber
        self.subscriber = rospy.Subscriber(
            rospy.get_param('~ball_subscribe_topic'),
            PoseWithCertaintyArray,
            self.ball_callback,
            queue_size=1
        )

        # publishes positons of ball
        self.ball_pose_publisher = rospy.Publisher(
            rospy.get_param('~ball_position_publish_topic'),
            PoseWithCovarianceStamped,
            queue_size=1
        )

        # publishes velocity of ball
        self.ball_movement_publisher = rospy.Publisher(
            rospy.get_param('~ball_movement_publish_topic'),
            TwistWithCovarianceStamped,
            queue_size=1
        )

        # publishes ball
        self.ball_publisher = rospy.Publisher(
            rospy.get_param('~ball_publish_topic'),
            PoseWithCertaintyStamped,
            queue_size=1
        )

        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(2))
        self.tf_listener = tf2.TransformListener(self.tf_buffer)

        self.filter_rate = rospy.get_param('~filter_rate')
        self.filter_time_step = 1.0 / self.filter_rate
        self.filter_reset_duration = rospy.Duration(secs=rospy.get_param('~filter_reset_time'))

        self.odom_frame = rospy.get_param('~odom_frame', 'odom')

        # adapt velocity factor to frequency
        self.velocity_factor = rospy.get_param('~velocity_reduction') ** (1 / self.filter_rate)

        self.filter_timer = rospy.Timer(rospy.Duration(self.filter_time_step), self.filter_step)
        rospy.spin()

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
                self.ball = self.tf_buffer.transform(ball_buffer, self.odom_frame, timeout=rospy.Duration(0.3))
                self.ball_header = msg.header
            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                rospy.logwarn(e)

    def filter_step(self, event):
        """"
        When ball has been assigned a value and filter has been initialized:
        state will be updated according to filter.
        If filter has not been initialized, then that will be done first.

        If there is not data for ball, a prediction will still be made and published
        Process noise is taken into account
        """
        if self.ball:
            if not self.filter_init:
                self.init_filter(*self.get_ball_measurement())
                self.filter_init = True
            self.kf.predict()
            self.kf.update(self.get_ball_measurement())
            state = self.kf.get_update()
            self.publish_data(*state)
            self.last_state = state
            self.ball = None
        else: 
            if self.filter_init:
                if (rospy.Time.now() - self.last_ball_msg_stamp) > self.filter_reset_duration:
                    self.filter_init = False
                    self.last_state = None
                    return
                self.kf.predict()
                self.kf.update(None)
                state = self.kf.get_update()
                self.publish_data(*state)
                self.last_state = state

    def get_ball_measurement(self):
        """extracts filter measurement from ball message"""
        return np.array([self.ball.point.x, self.ball.point.y])

    def init_filter(self, x, y):
        """
        initializes kalmanfilter at given position

        :param x: start x position of the ball
        :param y: start y position of the ball
        :return:
        """
        # initial value of position(x,y) of the ball and velocity
        self.kf.x = np.array([x, y, 0, 0])

        # transition matrix
        self.kf.F = np.array([[1.0, 0.0, self.velocity_factor, 0.0],
                             [0.0, 1.0, 0.0, self.velocity_factor],
                             [0.0, 0.0, 1.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]])
        # measurement function
        self.kf.H = np.array([[1.0, 0.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0, 0.0]])
        # multiplying by the initial uncertainty
        self.kf.P = np.eye(4) * 1000

        # assigning measurement noise
        self.kf.R = np.array([[1, 0],
                             [0, 1]]) * 0.1

        # assigning process noise
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=self.filter_time_step, var=rospy.get_param('~process_noise_variance'), block_size=2, order_by_dim=False)

    def publish_data(self, state: np.array, cov_mat: np.array) -> None:
        """
        Publishes ball position and velocity to ros nodes
        :param state: current state of kalmanfilter
        :param cov_mat: current covariance matrix
        :return:
        """
        # position
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = self.odom_frame
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
        movement_msg.twist.twist.linear.x = state[2]
        movement_msg.twist.twist.linear.y = state[3]
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
