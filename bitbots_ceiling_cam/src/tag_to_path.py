#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


rclpy.init(args=None)

pub = self.create_publisher(Path, "/robot_path", 1)

mypath = Path()
def tag_cb(msg):
    global mypath
    for d in msg.detections:
        if d.id[0] == 42:
            p = PoseStamped(d.pose.header, d.pose.pose.pose)
            mypath.poses.append(p)
            mypath.header = d.pose.header
            pub.publish(mypath)

rospy.Subscriber("/ceiling_cam/tag_detections", AprilTagDetectionArray, tag_cb)

while rclpy.ok():
    input("Press enter to reset path")
    mypath = Path()
    print("Path reset")

rclpy.spin(self)