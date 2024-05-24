from typing import Tuple

import cv2
import numpy as np
import rclpy
import rclpy.logging
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from soccer_vision_2d_msgs.msg import Robot, RobotArray

from bitbots_technical_challenge_vision.bitbots_technical_challenge_vision_params import (
    bitbots_technical_challenge_vision,
)


class TechnicalChallengeVision(Node):
    def __init__(self):
        super().__init__("bitbots_technical_challenge_vision")

        self._package_path = get_package_share_directory("bitbots_technical_challenge_vision")
        self._cv_bridge = CvBridge()
        self._annotations_pub = self.create_publisher(RobotArray, "/bitbots_technical_challenge_vision", 10)
        self._debug_img_pub = self.create_publisher(Image, "/bitbots_technical_challenge_vision_debug_img", 10)
        self._debug_clrmp_pub_blue = self.create_publisher(
            Image, "/bitbots_technical_challenge_vision_debug_clrmp_blue", 10
        )
        self._debug_clrmp_pub_red = self.create_publisher(
            Image, "/bitbots_technical_challenge_vision_debug_clrmp_red", 10
        )
        self._img_sub = self.create_subscription(Image, "/camera/image_proc", self.image_callback, 10)
        self._param_listener = bitbots_technical_challenge_vision.ParamListener(self)
        self._params = self._param_listener.get_params()
        self._timer = self.create_timer(1, self.timer_callback)

    def create_robot_msg(self, x: int, y: int, h: int, w: int, t: int) -> Robot:
        """
        Creates a Robot message from a robots bounding box data and its color.

        :param x: bb top left x
        :param y: bb top left y
        :param h: bb height
        :param w: bb width
        :param t: robot team
        :return: robot message for that robot
        """
        robot = Robot()

        robot.bb.center.position.x = float(x + (w / 2))
        robot.bb.center.position.y = float(y + (h / 2))
        robot.bb.size_x = float(w)
        robot.bb.size_y = float(h)
        robot.attributes.team = t

        return robot

    def process_image(
        self, img: np.ndarray, debug_img: np.ndarray, arg
    ) -> Tuple[list[Robot], list[Robot], np.ndarray, np.ndarray]:
        """
        gets annotations from the camera image

        :param img: ndarray containing the camera image
        :param debug_img: copy of img debug annotations should be drawn here
        :param arg: __RosParameters object containing the dynamic parameters
        :return: [[blue_robots], [red_robots], clrmp_blue, clrmp_red]
        """
        # convert img to hsv to isolate colors better
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

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
        blue_robots = []
        red_robots = []

        if arg.debug_mode:

            def annotate(x, y, h, w, c):
                return cv2.rectangle(
                    debug_img,
                    (x, y),
                    (x + w, y + h),
                    c,
                    2,
                )
        else:

            def annotate(x, y, h, w, c):
                return None

        for cnt in blue_contours:
            x, y, h, w = cv2.boundingRect(cnt)
            if min(h, w) >= arg.min_size and max(h, w) <= arg.max_size:
                # draw bb on debug img
                annotate(x, y, h, w, (255, 0, 0))

                # TODO I think 1 is for the blue team?
                blue_robots.append(self.create_robot_msg(x, y, h, w, 1))

        for cnt in red_contours:
            x, y, h, w = cv2.boundingRect(cnt)
            if min(h, w) >= arg.min_size and max(h, w) <= arg.max_size:
                # draw bb on debug img
                annotate(x, y, h, w, (0, 0, 255))

                red_robots.append(self.create_robot_msg(x, y, h, w, 2))

        return blue_robots, red_robots, blue_map, red_map, debug_img

    def image_callback(self, msg: Image):
        # get dynamic parameters
        arg = self._params

        # set variables
        img = self._cv_bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")
        header = msg.header

        if arg.debug_mode:
            debug_img = np.copy(img)
        else:
            debug_img = None

        # get annotations
        blue_robots, red_robots, blue_map, red_map, debug_img = self.process_image(img, debug_img, arg)
        robots = []
        robots.extend(blue_robots)
        robots.extend(red_robots)

        # make output message
        robot_array_message = RobotArray()
        robot_array_message.header = header
        robot_array_message.robots = robots

        # publish output message
        self._annotations_pub.publish(robot_array_message)

        if arg.debug_mode:
            # make debug image message
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

    def timer_callback(self):
        if self._param_listener.is_old(self._params):
            self._param_listener.refresh_dynamic_parameters()
            self._params = self._param_listener.get_params()


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
