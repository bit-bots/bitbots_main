import os
import threading

import cv2
import numpy as np
import rclpy
import yaml
from bitbots_utils.transforms import quat2fused
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from sensor_msgs.msg import Image
from soccer_vision_3d_msgs.msg import RobotArray
from transforms3d.affines import compose, decompose
from transforms3d.euler import euler2mat


class BFDataCollector(Node):
    def __init__(self):
        super().__init__("bf_data_collector")
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(Image, "/camera/image_proc", self.img_callback, 1)
        self.model_state_sub = self.create_subscription(ModelStates, "/model_states", self.model_state_callback, 1)
        self.robot_relative_sub = self.create_subscription(
            RobotArray, "/robots_relative", self.robot_relative_callback, 1
        )
        self.img = None
        self.own_robot_pose = None
        self.other_robot_pose = None
        self.other_robot_name = "ROBOTIS OP3"
        self.relative_base_footprint = None
        self.relative_baseline = None
        self.sample_index = 0
        dir_name = "testlog1"  # input("Enter directory name: ")
        self.path = f"/homes/15guelden/saved_data/{dir_name}"
        os.makedirs(self.path, exist_ok=True)
        input("enter to start collecting datas")

    def loop(self):
        while rclpy.ok():
            input("Press Enter to save data")
            self.save_data()

    def save_data(self):
        with open(self.path + f"/{self.sample_index:03d}_data.yaml", "w") as f:
            fused_roll, fused_pitch, fused_yaw, hemi = quat2fused(
                [
                    self.own_robot_pose.orientation.x,
                    self.own_robot_pose.orientation.y,
                    self.own_robot_pose.orientation.z,
                    self.own_robot_pose.orientation.w,
                ],
                order="xyzw",
            )
            world_to_own_robot = compose(
                [self.own_robot_pose.position.x, self.own_robot_pose.position.y, 0],
                euler2mat(0, 0, fused_yaw, axes="sxyz"),
                np.ones(3),
            )
            world_to_other_robot = compose(
                [self.other_robot_pose.position.x, self.other_robot_pose.position.y, 0],
                euler2mat(0, 0, 0, axes="sxyz"),
                np.ones(3),
            )
            t_bf_other_robot = np.linalg.inv(world_to_own_robot) @ world_to_other_robot
            translation, _, _, _ = decompose(t_bf_other_robot)
            data = {
                "ground_truth_robot_pose": {
                    "x": float(translation[0]),
                    "y": float(translation[1]),
                    "d": float(np.linalg.norm(translation[:2])),
                },
                "relative_base_footprint": {
                    "x": self.relative_base_footprint.x,
                    "y": self.relative_base_footprint.y,
                    "d": float(np.linalg.norm([self.relative_base_footprint.x, self.relative_base_footprint.y])),
                },
                "relative_baseline": {
                    "x": self.relative_baseline.x,
                    "y": self.relative_baseline.y,
                    "d": float(np.linalg.norm([self.relative_baseline.x, self.relative_baseline.y])),
                },
            }
            yaml.dump(data, f)
        cv2.imwrite(self.path + f"/{self.sample_index:03d}_img.jpg", self.img)
        self.sample_index += 1

    def img_callback(self, img_msg: Image):
        self.img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

    def model_state_callback(self, model_state_msg: ModelStates):
        for name, pose in zip(model_state_msg.name, model_state_msg.pose):
            if name == self.other_robot_name:
                self.other_robot_pose = pose
            if name == "amy":
                self.own_robot_pose = pose

    def robot_relative_callback(self, robot_relative_msg: RobotArray):
        if len(robot_relative_msg.robots) < 2:
            return
        for robot in robot_relative_msg.robots[:2]:
            if robot.attributes.team == 1:
                self.relative_base_footprint = robot.bb.center.position
            elif robot.attributes.team == 0:
                self.relative_baseline = robot.bb.center.position


def main(args=None):
    rclpy.init()
    bf_data_collector = BFDataCollector()
    thread = threading.Thread(target=rclpy.spin, args=(bf_data_collector,))
    thread.start()
    bf_data_collector.loop()
    bf_data_collector.destroy_node()
    rclpy.shutdown()
