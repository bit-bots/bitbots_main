#! /usr/bin/env python3
import rclpy
import numpy as np
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped


class SimBall:
    def __init__(self):
        rclpy.init(args=sys.argv)
        node = rclpy.create_node('ball_sim_node')
        node.get_logger().info('Created ball_sim_node')

        self.pub_frequency = 20
        if len(sys.argv) > 1:
            self.pub_frequency = int(sys.argv[1])
        self.dt = 1.0 / self.pub_frequency

        self.max_acceleration = .8
        self.max_error = np.array([2, 4])

        self.velocity = np.zeros((2))
        self.position = np.zeros((2))
        self.p_pub = node.create_publisher(PoseWithCovarianceStamped, 'position', 1)
        self.p_err_pub = node.create_publisher(PoseWithCovarianceStamped, 'ball_relative', 1)
        rate = node.create_rate(self.pub_frequency)
        while not rclpy.ok():
            self.position += self.velocity * self.dt
            self.velocity = np.clip(self.velocity + self.gen_acceleration(), -3, 3)
            p_err = self.position + self.gen_error()
            self.p_pub.publish(self.gen_msg(self.position[0], self.position[1]))
            self.p_err_pub.publish(self.gen_msg(p_err[0], p_err[1]))
            rate.sleep()

    def gen_error(self):
        return np.multiply(np.random.rand(2) * 2 - 1, self.max_error)

    def gen_acceleration(self):
        return np.multiply(np.random.randn(2), self.max_acceleration) * self.dt

    def gen_msg(self, x, y):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'world'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1
        return msg


if __name__ == '__main__':
    rclpy.spin(SimBall())



