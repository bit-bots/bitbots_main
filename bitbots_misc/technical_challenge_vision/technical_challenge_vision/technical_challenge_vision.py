from typing import Tuple

import cv2
import numpy as np
import rclpy
import rclpy.logging
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from bitbots_msgs.msg import BoundingBox, BoundingBoxArray
from technical_challenge_vision.technical_challenge_vision_params import technical_challenge_vision


class TechnicalChallengeVision(Node):
    def __init__(self):
        super().__init__("technical_challenge_vision")

        self._package_path = get_package_share_directory("technical_challenge_vision")
        self._cv_bridge = CvBridge()
        self._annotations_pub = self.create_publisher(BoundingBoxArray, "/technical_challenge_vision", 10)
        self._debug_img_pub = self.create_publisher(Image, "/technical_challenge_vision_debug_img", 10)
        self._debug_clrmp_pub_blue = self.create_publisher(Image, "/technical_challenge_vision_debug_clrmp_blue", 10)
        self._debug_clrmp_pub_red = self.create_publisher(Image, "/technical_challenge_vision_debug_clrmp_red", 10)
        self._img_sub = self.create_subscription(Image, "/camera/image_proc", self.image_callback, 10)
        self._param_listener = technical_challenge_vision.ParamListener(self)
        self._params = self._param_listener.get_params()

    def process_image(self, img: np.ndarray) -> Tuple[list[BoundingBox], list[BoundingBox], np.ndarray, np.ndarray]:
        """
        gets annotations from the camera image

        :param img: ndarray containing the camera image
        """
        # convert img to hsv to isolate colors better
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # get dynamic parameters
        arg = self._params.ros__parameters

        # get color maps
        blue_map = cv2.inRange(
            img,
            (arg.blue_lower_h, arg.blue_lower_s, arg.blue_lower_v),
            (arg.blue_upper_h, arg.blue_upper_s, arg.blue_upper_v),
        )

        red_map = cv2.inRange(
            img,
            (arg.red_lower_h, arg.red_lower_s, arg.red_lower_v),
            (arg.red_upper_h, arg.red_upper_s, arg.red_upper_v),
        )

        # get contours in color maps
        blue_contours, _ = cv2.findContours(blue_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        red_contours, _ = cv2.findContours(red_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # get lists of bounding boxes
        blue_boxes = []
        red_boxes = []

        for cnt in blue_contours:
            x, y, h, w = cv2.boundingRect(cnt)
            if min(h, w) >= arg.min_size and max(h, w) <= arg.max_size:
                blue_boxes.append(BoundingBox(top_left_x=x, top_left_y=y, height=h, width=w))

        for cnt in red_contours:
            x, y, h, w = cv2.boundingRect(cnt)
            if min(h, w) >= arg.min_size and max(h, w) <= arg.max_size:
                red_boxes.append(BoundingBox(top_left_x=x, top_left_y=y, height=h, width=w))

        return blue_boxes, red_boxes, blue_map, red_map

    def image_callback(self, msg: Image):
        # get image from message
        img = self._cv_bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")
        header = msg.header

        # get annotations
        blue_boxes, red_boxes, blue_map, red_map = self.process_image(img)

        # make output message
        bb_message = BoundingBoxArray()
        bb_message.header = header
        bb_message.blue_boxes = blue_boxes
        bb_message.red_boxes = red_boxes

        # publish output message
        self._annotations_pub.publish(bb_message)

        # draw on debug image
        debug_img = np.copy(img)

        for bb in blue_boxes:
            debug_img = cv2.rectangle(
                debug_img,
                (bb.top_left_x, bb.top_left_y),
                (bb.top_left_x + bb.width, bb.top_left_y + bb.height),
                (255, 0, 0),
                2,
            )

        for bb in red_boxes:
            debug_img = cv2.rectangle(
                debug_img,
                (bb.top_left_x, bb.top_left_y),
                (bb.top_left_x + bb.width, bb.top_left_y + bb.height),
                (0, 0, 255),
                2,
            )

        # make debug image message
        # debug_img = cv2.cvtColor(debug_img, cv2.COLOR_BGR2RGB)
        debug_img_msg = self._cv_bridge.cv2_to_imgmsg(cvim=debug_img, encoding="bgr8", header=header)

        # publish debug image
        self._debug_img_pub.publish(debug_img_msg)

        # make color map messages
        clrmp_blue_img = cv2.cvtColor(blue_map, cv2.COLOR_GRAY2BGR)
        clrmp_blue_msg = self._cv_bridge.cv2_to_imgmsg(cvim=clrmp_blue_img, encoding="bgr8", header=header)

        clrmp_red_img = cv2.cvtColor(red_map, cv2.COLOR_GRAY2BGR)
        clrmp_red_msg = self._cv_bridge.cv2_to_imgmsg(clrmp_red_img, encoding="bgr8", header=header)

        # publish color map messages
        self._debug_clrmp_pub_blue.publish(clrmp_blue_msg)
        self._debug_clrmp_pub_red.publish(clrmp_red_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TechnicalChallengeVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
