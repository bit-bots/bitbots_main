#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
import tf2_ros as tf2



class TFWorld(object):
    def __init__(self):
        rclpy.init()
        node = Node("map")
        br = tf2.TransformBroadcaster(node)
        r = node.create_rate(100)
        while rclpy.ok():
            transform = TransformStamped()
            transform.header.frame_id = "map"
            transform.header.stamp = (node.get_clock().now() + Duration(seconds=0, nanoseconds=0.1*10e6)).to_msg()
            transform.child_frame_id = "odom"

            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            br.sendTransform(transform)
            rclpy.spin_once(node)
            r.sleep()
            


if __name__ == "__main__":
    TFWorld()
