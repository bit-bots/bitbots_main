#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np


class LocalizationFaker(Node):

    def __init__(self):
        super().__init__('localization_faker')
        self.create_subscription(ModelStates, "/model_states", self.model_state_to_tf, 10)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)

    def model_state_to_tf(self, model_state_msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now()
        for i, name in enumerate(model_state_msg.name):
            t.header.frame_id = name + "/map"
            t.child_frame_id = name + "/odom"
            try:
                robot_in_odom = self.tf_buffer.lookup_transform(name + "/odom",
                                                                name + "/base_link",
                                                                t.header.stamp,
                                                                timeout=Duration(seconds=0.1))
            except tf2_ros.LookupException as ex:
                self.get_logger().warn(self.get_name() + ": " + str(ex), throttle_duration_sec=5.0)
                return
            except tf2_ros.ExtrapolationException as ex:
                self.get_logger().warn(self.get_name() + ": " + str(ex), throttle_duration_sec=5.0)
                return
            rot_odom = transforms3d.quaternions.quat2mat([robot_in_odom.transform.rotation.w,
                                                          robot_in_odom.transform.rotation.x,
                                                          robot_in_odom.transform.rotation.y,
                                                          robot_in_odom.transform.rotation.z])

            transform_odom = transforms3d.affines.compose([robot_in_odom.transform.translation.x,
                                                           robot_in_odom.transform.translation.y,
                                                           robot_in_odom.transform.translation.z], rot_odom, [1, 1, 1])

            pos_robot = [model_state_msg.pose[i].position.x, model_state_msg.pose[i].position.y,
                         model_state_msg.pose[i].position.z]
            rot_robot = transforms3d.quaternions.quat2mat([model_state_msg.pose[i].orientation.w,
                                                           model_state_msg.pose[i].orientation.x,
                                                           model_state_msg.pose[i].orientation.y,
                                                           model_state_msg.pose[i].orientation.z])
            transform_robot = transforms3d.affines.compose(pos_robot, rot_robot, [1, 1, 1])

            transform_final = np.matmul(transform_robot, np.linalg.inv(transform_odom))
            T, R, _, _ = transforms3d.affines.decompose(transform_final)
            t.transform.translation.x = T[0]
            t.transform.translation.y = T[1]
            t.transform.translation.z = T[2]
            q = transforms3d.quaternions.mat2quat(R)
            t.transform.rotation.w = q[0]
            t.transform.rotation.x = q[1]
            t.transform.rotation.y = q[2]
            t.transform.rotation.z = q[3]
            self.br.sendTransform(t)


if __name__ == "__main__":
    rclpy.init(args=None)
    node = LocalizationFaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
