#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped
from humanoid_league_msgs.msg import PoseWithCertainty, PoseWithCertaintyArray


class SimBall(Node):
    def __init__(self):
        super().__init__('ball_sim_node')
        self.get_logger().info('Created ball_sim_node')

        self.pub_frequency = 20
        # if len(sys.argv) > 1:
        #     self.pub_frequency = int(sys.argv[1])
        self.dt = 1.0 / self.pub_frequency

        self.max_acceleration = .8
        self.max_error = np.array([2, 4])

        self.velocity = np.zeros((2))
        self.position = np.zeros((2))
        self.p_pub = self.create_publisher(PoseWithCovarianceStamped, 'position', 1)
        self.p_err_pub = self.create_publisher(PoseWithCertaintyArray, 'balls_relative', 1)
        self.timer = self.create_timer(self.dt, self.step)

    def step(self):
        self.position += self.velocity * self.dt
        self.velocity = np.clip(self.velocity + self.gen_acceleration(), -3, 3)
        p_err = self.position + self.gen_error()
        self.p_pub.publish(self.gen_msg(self.position[0], self.position[1]))
        self.p_err_pub.publish(self.gen_err_msg(p_err[0], p_err[1]))
    
    def gen_error(self):
        return np.multiply(np.random.rand(2) * 2 - 1, self.max_error)

    def gen_acceleration(self):
        return np.multiply(np.random.randn(2), self.max_acceleration) * self.dt

    def gen_msg(self, x, y):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation.w = 1.0
        return pose_msg

    def gen_err_msg(self, x, y):
        pose_msg = PoseWithCertainty()
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation.w = 1.0
        pose_msg.confidence = 0.9

        poses_msg = PoseWithCertaintyArray()
        poses_msg.header.frame_id = 'odom'
        poses_msg.poses.append(pose_msg)
        return poses_msg


def main(args=None):
    rclpy.init(args=args)
    node = SimBall()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
