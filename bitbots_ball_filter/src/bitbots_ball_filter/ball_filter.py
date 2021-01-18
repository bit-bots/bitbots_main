#! /usr/bin/env python3

import rospy
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped


class BallFilter:
    def __init__(self):
        """
        creates Kalmanfilter and subscribes to messages which are needed
        """
        rospy.init_node('ball_filter')
        #creates kalmanfilter with 4 dimensions
        self.kf = KalmanFilter(dim_x=4, dim_z=2, dim_u=0)
        self.filter_init = False
        self.ball = None  # type:PoseWithCovarianceStamped
        self.ball_header = None

        # setup subscriber
        self.subscriber = rospy.Subscriber(
            rospy.get_param('/ball_filter/ball_subscribe_topic'),
            PoseWithCovarianceStamped,
            self.ball_callback,
            queue_size=1
        )
        #publishes positons of ball
        self.ball_pose_publisher = rospy.Publisher(
            rospy.get_param('/ball_filter/ball_position_publish_topic'),
            PoseWithCovarianceStamped,
            queue_size=1
        )
        #publishes velocity of ball
        self.ball_movement_publisher = rospy.Publisher(
            rospy.get_param('/ball_filter/ball_movement_publish_topic'),
            TwistWithCovarianceStamped,
            queue_size=1
        )

        self.filter_time_step = rospy.get_param('/ball_filter/filter_time_step')

        self.filter_timer = rospy.Timer(rospy.Duration(self.filter_time_step), self.filter_step)
        rospy.spin()

    def ball_callback(self, msg: PoseWithCovarianceStamped):
        """assigns position of ball to self.ball and the header to ball_header"""
        self.ball = msg
        self.ball_header = msg.header

    def filter_step(self, event):
        """"When ball has been assigned a value and filter has been initialized:
            state will be updated according to filter.
            If filter has not been initialized, then that will be done first.

            If there is not data for ball, a prediction will still be made and published
            Process noise is taken into account
            """
        if self.ball:
            if self.filter_init:
                self.kf.predict()
                self.kf.update(self.get_ball_measurement())
                state = self.kf.get_update()
            else:
                self.init_filter(*self.get_ball_measurement())
                self.kf.predict()
                self.kf.update(self.get_ball_measurement())
                state = self.kf.get_update()
                self.filter_init = True
            self.publish_data(*state)
            self.ball = None
        else: 
            if self.filter_init:
                self.kf.predict()
                self.kf.update(None)
                state = self.kf.get_update()
                self.publish_data(*state)

    def get_ball_measurement(self):
        """extracts filter measurement from ball message"""
        return np.array([self.ball.pose.pose.position.x, self.ball.pose.pose.position.y])

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
        self.kf.F = np.array([[1.0, 0.0, 1.0, 0.0],
                             [0.0, 1.0, 0.0, 1.0],
                             [0.0, 0.0, 1.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]])
        # measurement function
        self.kf.H = np.array([[1.0, 0.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0, 0.0]])
        # multiplying by the initial uncertainty
        self.kf.P = np.eye(4) * 1000
        # assigning measurement noise
        # TODO: paramter
        self.kf.R = np.array([[1, 0],
                             [0, 1]]) * 0.1
        # assigning process noise
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=self.filter_time_step, var=rospy.get_param('/ball_filter/process_noise_variance'), block_size=2, order_by_dim=False)

    def publish_data(self, state: np.array, cov_mat: np.array) -> None:
        """
        Publishes ball position and velocity to ros nodes
        :param state: current state of kalmanfilter
        :param cov_mat: current covariance matrix
        :return:
        """
        header = self.ball_header
        header.stamp = rospy.Time.now()
        # position
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = header
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
        movement_msg.header = header
        movement_msg.twist.twist.linear.x = state[2]
        movement_msg.twist.twist.linear.y = state[3]
        movement_msg.twist.covariance = np.eye(6).reshape((36))
        movement_msg.twist.covariance[0] = cov_mat[2][2]
        movement_msg.twist.covariance[1] = cov_mat[2][3]
        movement_msg.twist.covariance[6] = cov_mat[3][2]
        movement_msg.twist.covariance[7] = cov_mat[3][3]
        self.ball_movement_publisher.publish(movement_msg)


if __name__ == "__main__":
    BallFilter()
