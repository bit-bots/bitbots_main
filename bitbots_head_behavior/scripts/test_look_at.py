#!/usr/bin/env python3
from __future__ import print_function
import rclpy
from rcply.duration import Duration
from rclpy.node import Node
import tf2_ros as tf2
from bio_ik_msgs.msg import IKRequest, LookAtGoal
from bio_ik_msgs.srv import GetIK
from bitbots_msgs.msg import JointCommand
from geometry_msgs.msg import Point
from time import sleep
from tf2_geometry_msgs import PointStamped

if __name__ == "__main__":
    rclpy.init(args=None)
    node = Node("test_look_at")
    base_footprint_frame = node.get_parameter('~base_footprint_frame').get_parameter_value().double_value
    camera_frame = node.get_parameter('~camera_frame').get_parameter_value().double_value
    base_link_frame = node.get_parameter('~base_link_frame').get_parameter_value().double_value

    head_tf_frame = base_link_frame
    tf_buffer = tf2.Buffer(Duration(seconds=5))
    tf_listener = tf2.TransformListener(tf_buffer)

    request = IKRequest()
    request.group_name = "Head"
    request.timeout.secs = 1
    request.approximate = True
    request.look_at_goals.append(LookAtGoal())
    request.look_at_goals[0].link_name = camera_frame
    request.look_at_goals[0].weight = 1
    request.look_at_goals[0].axis.x = 1

    pos_msg = JointCommand()
    pos_msg.joint_names = ["HeadPan", "HeadTilt"]
    pos_msg.positions = [0, 0]
    pos_msg.velocities = [1.5, 1.5]
    pos_msg.accelerations = [-1, -1]
    pos_msg.max_currents = [-1, -1]

    rclpy.wait_for_service("bio_ik/get_bio_ik")
    bio_ik = node.create_client(GetIK, 'bio_ik/get_bio_ik')

    publish_motor_goals = node.create_publisher(JointCommand, 'head_motor_goals', 10)

    while rclpy.ok():
        x = float(input('x: '))
        y = float(input('y: '))
        point = PointStamped()
        point.header.stamp = node.get_clock().now().to_msg()
        point.header.frame_id = base_footprint_frame
        point.point.x = x
        point.point.y = y
        point.point.z = 0

        ###### Movement ######
        point = tf_buffer.transform(point, head_tf_frame, timeout=Duration(seconds=0.3)).point
        target = Point(point.x, point.y, point.z)
        request.look_at_goals[0].target = target
        response = bio_ik(request).ik_response
        states = response.solution.joint_state
        head_pan = states.position[states.name.index('HeadPan')]
        head_tilt = states.position[states.name.index('HeadTilt')]

        print("Motor positions {}, {}".format(head_pan, head_tilt))
        pos_msg.positions = [head_pan, head_tilt]
        pos_msg.header.stamp = node.get_clock().now().to_msg()
        publish_motor_goals.publish(pos_msg)

        # Sleep 1 to wait for head being at its position
        sleep(1)

        ###### Validation ######
        # Reverse read position where the robot is looking at
        # Get line of camera
        camera_origin = PointStamped()
        camera_origin.header.stamp = node.get_clock().now().to_msg()
        camera_origin.header.frame_id = camera_frame
        camera_origin.point = Point(0, 0, 0)
        camera_one = PointStamped()
        camera_one.header.stamp = node.get_clock().now().to_msg()
        camera_one.header.frame_id = camera_frame
        camera_one.point = Point(1, 0, 0)
        camera_origin_bf = tf_buffer.transform(camera_origin, base_footprint_frame, timeout=Duration(seconds=0.3)).point
        camera_one_bf = tf_buffer.transform(camera_one, base_footprint_frame, timeout=Duration(seconds=0.3)).point
        line_vector = Point(camera_one_bf.x - camera_origin_bf.x,
                            camera_one_bf.y - camera_origin_bf.y,
                            camera_one_bf.z - camera_origin_bf.z)
        # Calculate where it touches the plane of base_footprint
        # line will be described as camera_one_bf + u * line_vector, where u is a scalar
        # now the line should equal (x y 0)
        u = - camera_origin_bf.z / line_vector.z
        # calculate x and y
        x = camera_origin_bf.x + u * line_vector.x
        y = camera_origin_bf.y + u * line_vector.y
        # Give result in base_footprint frame
        print("Actually looking at {}, {}".format(x, y))
