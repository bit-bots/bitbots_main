import math
import rclpy
from rclpy.node import Node
from humanoid_league_msgs.msg import (PoseWithCertainty,
                                      PoseWithCertaintyArray)
"""
Node that can be used to publish a circling dummy ball for the ball filter
"""
class BP(Node):
    def __init__(self):
        super().__init__("ball_filter_helper")

        # setup publisher to publish same ball
        self.ball_publisher = self.create_publisher(
            PoseWithCertaintyArray,
            'balls_relative',
            1
        )
        rate = self.create_timer(0.1, self.publish_ball)

        self.counter = 0


    def publish_ball(self):
        array = PoseWithCertaintyArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = "odom"
        pose = PoseWithCertainty()
        pose.pose.pose.position.x = 1.0 + math.sin(self.counter)
        pose.pose.pose.position.y = 1.0 + math.cos(self.counter)
        self.counter += 0.1
        pose.confidence = 1.0
        array.poses = [pose]
        self.ball_publisher.publish(array)

def main(args=None):
    rclpy.init(args=args)
    node = BP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()