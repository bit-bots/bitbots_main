#!/usr/bin/env python3

import numpy as np
import rclpy
import tf2_ros
import transforms3d
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter


class LocalizationFaker(Node):
    def __init__(self):
        super().__init__("localization_faker")
        self.create_subscription(ModelStates, "/model_states", self.model_state_to_tf, 10)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        use_sim_time_param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        self.set_parameters([use_sim_time_param])

    def model_state_to_tf(self, model_state_msg):
        t = TransformStamped()
        try:
            time = self.get_clock().now() - Duration(seconds=0.5)
        except Exception:
            return
        t.header.stamp = time.to_msg()
        for i, name in enumerate(model_state_msg.name):
            if name != "amy":
                continue
            t.header.frame_id = "map"
            t.child_frame_id = "odom"
            try:
                robot_in_odom = self.tf_buffer.lookup_transform(
                    "odom", "base_link", t.header.stamp, timeout=Duration(seconds=0.1)
                )
            except tf2_ros.LookupException as ex:
                self.get_logger().warn(self.get_name() + ": " + str(ex), throttle_duration_sec=5.0)
                return
            except tf2_ros.ExtrapolationException as ex:
                self.get_logger().warn(self.get_name() + ": " + str(ex), throttle_duration_sec=5.0)
                return
            rot_odom = transforms3d.quaternions.quat2mat(
                [
                    robot_in_odom.transform.rotation.w,
                    robot_in_odom.transform.rotation.x,
                    robot_in_odom.transform.rotation.y,
                    robot_in_odom.transform.rotation.z,
                ]
            )

            transform_odom = transforms3d.affines.compose(
                [
                    robot_in_odom.transform.translation.x,
                    robot_in_odom.transform.translation.y,
                    robot_in_odom.transform.translation.z,
                ],
                rot_odom,
                [1, 1, 1],
            )

            pos_robot = [
                model_state_msg.pose[i].position.x,
                model_state_msg.pose[i].position.y,
                model_state_msg.pose[i].position.z,
            ]
            rot_robot = transforms3d.quaternions.quat2mat(
                [
                    model_state_msg.pose[i].orientation.w,
                    model_state_msg.pose[i].orientation.x,
                    model_state_msg.pose[i].orientation.y,
                    model_state_msg.pose[i].orientation.z,
                ]
            )
            transform_robot = transforms3d.affines.compose(pos_robot, rot_robot, [1, 1, 1])

            transform_final = np.matmul(transform_robot, np.linalg.inv(transform_odom))
            translation, r, _, _ = transforms3d.affines.decompose(transform_final)
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            q = transforms3d.quaternions.mat2quat(r)
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
