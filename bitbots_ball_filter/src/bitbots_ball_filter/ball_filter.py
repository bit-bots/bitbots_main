#! /usr/bin/env python3

import rospy
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped


class BallFilter:
    def __init__(self):
        rospy.init_node('ball_filter')

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

        self.ball_pose_publisher = rospy.Publisher(
            rospy.get_param('/ball_filter/ball_position_publish_topic'),
            PoseWithCovarianceStamped,
            queue_size=1
        )
        self.ball_movement_publisher = rospy.Publisher(
            rospy.get_param('/ball_filter/ball_movement_publish_topic'),
            TwistWithCovarianceStamped,
            queue_size=1
        )

        self.filter_time_step = rospy.get_param('/ball_filter/filter_time_step')

        self.filter_timer = rospy.Timer(rospy.Duration(self.filter_time_step), self.filter_step)
        rospy.spin()

    def ball_callback(self, msg: PoseWithCovarianceStamped):
        self.ball = msg
        self.ball_header = msg.header

    def filter_step(self, event):
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
        return np.array([self.ball.pose.pose.position.x, self.ball.pose.pose.position.y])

    def init_filter(self, x, y):
        self.kf.x = np.array([x, y, 0, 0])
        self.kf.F = np.array([[1.0, 0.0, 1.0, 0.0],
                             [0.0, 1.0, 0.0, 1.0],
                             [0.0, 0.0, 1.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]])
        self.kf.H = np.array([[1.0, 0.0 , 0.0, 0.0],
                              [0.0, 1.0 , 0.0, 0.0]])
        self.kf.P = np.eye(4) * 1000
        self.kf.R = np.array([[1, 0],
                             [0, 1]]) * 0.1
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=self.filter_time_step, var=rospy.get_param('/ball_filter/process_noise_variance'), block_size=2, order_by_dim=False)

    def publish_data(self, state: np.array, cov_mat: np.array) -> None:
        header = self.ball_header
        header.stamp = rospy.Time.now()
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
