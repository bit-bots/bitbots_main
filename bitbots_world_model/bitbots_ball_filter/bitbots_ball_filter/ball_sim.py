#! /usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from soccer_vision_3d_msgs.msg import Ball, BallArray


class SimBall(Node):
    def __init__(self):
        super().__init__("ball_sim_node")
        self.logger = self.get_logger()
        self.logger.info("Created ball_sim_node")

        # Simulation timings
        self.pub_frequency = 20
        self.dt = 1.0 / self.pub_frequency

        # Max ball movements
        self.max_velocity = np.array([2, 2])  # m/s
        self.max_acceleration = np.array([0.8, 0.8])  # m/s/s
        self.max_error = np.array([1, 1])

        # Initialize velocity and position
        self.velocity = np.zeros(2)
        self.position = np.zeros(2)

        # Create publishers and timer
        self.pub_pos_viz = self.create_publisher(PoseWithCovarianceStamped, "position", 1)
        self.pub_pos_err_viz = self.create_publisher(PoseWithCovarianceStamped, "position_err", 1)
        self.pub_pos_err = self.create_publisher(BallArray, "balls_relative", 1)
        self.timer = self.create_timer(self.dt, self.step)

    def step(self):
        # Step the ball
        self.velocity = np.clip(
            self.velocity + self.gen_acceleration() * self.dt, -self.max_velocity, self.max_velocity
        )
        self.position += self.velocity * self.dt
        p_err = self.position + self.gen_error()

        # Publish results
        self.pub_pos_viz.publish(self.gen_pose_cov_stamped_msg(self.position[0], self.position[1]))
        self.pub_pos_err_viz.publish(self.gen_pose_cov_stamped_msg(p_err[0], p_err[1]))
        self.pub_pos_err.publish(self.gen_ball_array_msg(p_err[0], p_err[1]))

    def gen_error(self):
        return np.multiply(np.random.rand(2) * 2 - 1, self.max_error)

    def gen_acceleration(self):
        return np.clip(np.random.randn(2), -self.max_acceleration, self.max_acceleration)

    def gen_pose_cov_stamped_msg(self, x, y):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rclpy.time.Time.to_msg(self.get_clock().now())
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation.w = 1.0
        return pose_msg

    def gen_ball_array_msg(self, x, y):
        ball_msg = Ball()
        ball_msg.center.x = x
        ball_msg.center.y = y
        ball_msg.confidence.confidence = 0.9

        ball_array_msg = BallArray()
        ball_array_msg.header.stamp = rclpy.time.Time.to_msg(self.get_clock().now())
        ball_array_msg.header.frame_id = "odom"
        ball_array_msg.balls.append(ball_msg)
        return ball_array_msg


def main(args=None):
    rclpy.init(args=args)
    node = SimBall()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
