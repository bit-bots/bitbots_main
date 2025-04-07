#! /usr/bin/env python3
from typing import Dict

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from soccer_vision_2d_msgs.msg import Robot, RobotArray
from yoeo import detect, models

# from bitbots_vision.ros_utils import build_bounding_box_2d, build_robot_msg, build_robot_array_msg


class YOEOVision(Node):
    def __init__(self) -> None:
        super().__init__("bitbots_vision")

        self._config: Dict = {}
        self._cv_bridge = CvBridge()

        self._sub_image = None
        self._sub_image = self.create_subscription(
            Image,
            "/camera/image_proc",
            self.image_cb,
            1,
        )
        self.model = models.load_model(
            "/homes/15guelden/bf_yoeo/yoeo.cfg",
            "/homes/15guelden/bf_yoeo/yoeo_checkpoint_500.pth",
        )
        print("model loaded")
        # debug image publisher
        self.debug_img_pub = self.create_publisher(Image, "/debug_image", 1)
        self.robot_pub = self.create_publisher(RobotArray, "/robots_in_image", 1)

    def image_cb(self, img_msg: Image):
        img = self._cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        boxes, _ = detect.detect_image(self.model, img, conf_thres=0.25)
        debug_img = img.copy()
        for box in boxes:
            debug_img = cv2.rectangle(debug_img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
            if box[6] > 0.5:
                debug_img = cv2.circle(debug_img, (int(box[7]), int(box[8])), 5, (255, 0, 255), -1)
        debug_img_msg = self._cv_bridge.cv2_to_imgmsg(debug_img, "bgr8")
        self.debug_img_pub.publish(debug_img_msg)
        robot_array_msg = RobotArray()
        robot_array_msg.header = img_msg.header
        robot_array_msg.robots = []
        for i, box in enumerate(boxes):
            if box[5] == 1.0 or box[5] == 2.0 or box[5] == 3.0:
                robot_msg_baseline = Robot()
                robot_msg_baseline.bb.center.position.x = float((box[0] + box[2]) / 2)
                robot_msg_baseline.bb.center.position.y = float((box[1] + box[3]) / 2)
                robot_msg_baseline.bb.size_x = float(box[2] - box[0])
                robot_msg_baseline.bb.size_y = float(box[3] - box[1])
                robot_msg_baseline.attributes.player_number = i
                robot_msg_baseline.attributes.team = 0
                robot_array_msg.robots.append(robot_msg_baseline)

                robot_msg_bf = Robot()
                robot_msg_bf.bb.center.position.x = float(box[7])
                robot_msg_bf.bb.center.position.y = float(box[8])
                robot_msg_bf.bb.size_x = 0.0
                robot_msg_bf.bb.size_y = 0.0
                robot_msg_bf.attributes.player_number = i
                robot_msg_bf.attributes.team = 1
                robot_array_msg.robots.append(robot_msg_bf)
        self.robot_pub.publish(robot_array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOEOVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
