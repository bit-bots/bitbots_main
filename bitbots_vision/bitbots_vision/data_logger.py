from copy import deepcopy
from typing import Dict, List
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from soccer_vision_3d_msgs.msg import RobotArray
from gazebo_msgs.msg import ModelStates
import threading
import os
import json
import csv
import yaml

class BFDataCollector(Node):
    def __init__(self):
        super().__init__('bf_data_collector')
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(
            Image,
            '/camera/image_processed',
            self.img_callback,
            1
        )
        self.model_state_sub = self.create_subscription(
            ModelStates,
            '/model_state',
            self.model_state_callback,
            1
        )
        self.robot_relative_sub = self.create_subscription(
            RobotArray,
            '/robot_relative',
            self.robot_relative_callback,
            1
        )
        self.img = None
        self.own_robot_pose = None
        self.other_robot_pose = None
        self.other_robot_name = "ROBOTIS OP3"
        self.relative_base_footprint = None
        self.relative_baseline = None
        self.sample_index = 0

    def loop(self):
        while rclpy.ok():
            input("Press Enter to save data")
            self.save_data()


    def save_data(self):
        with open(f'/homes/21stahl/Desktop/saved_data/saved_data{self.sample_index}.yaml', 'w') as f:
            data = {
                "img": self.img.tolist(),
                "own_robot_pose": {"position": {"x": self.own_robot_pose.position.x, "y": self.own_robot_pose.position.y},
                                   "orientation": {"x": self.own_robot_pose.orientation.x, "y": self.own_robot_pose.orientation.y,"w": self.own_robot_pose.orientation.w}},

                "other_robot_pose": {"position": {"x": self.other_robot_pose.position.x, "y": self.other_robot_pose.position.y},},
                "relative_base_footprint": {"x": self.relative_base_footprint.x, "y": self.relative_base_footprint.y},
                "relative_baseline": {"x": self.relative_baseline.x, "y": self.relative_baseline.y}
            }
            yaml.dump(data, f)
        self.sample_index += 1

    def img_callback(self, img_msg : Image):
        self.img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

    def model_state_callback(self, model_state_msg : ModelStates):
        for name, pose in zip(model_state_msg.name, model_state_msg.pose):
            if name == self.other_robot_name:
                self.other_robot_pose = pose
            if name == "amy":
                self.own_robot_pose = pose
            
    def robot_relative_callback(self, robot_relative_msg : RobotArray):
        if len(robot_relative_msg.robots) < 2:
            return
        for robot in robot_relative_msg.robots[:2]:
            if robot.attributes.team == 1:
                self.relative_base_footprint = robot.bb.center.position
            elif robot.attributes.team == 2:
                self.relative_baseline = robot.bb.center.position
    


if __name__ == "__main__":
    rclpy.init()
    bf_data_collector = BFDataCollector()
    thread = threading.Thread(target=rclpy.spin, args=(bf_data_collector,))
    thread.start()
    bf_data_collector.loop()
    bf_data_collector.destroy_node()
    rclpy.shutdown()
