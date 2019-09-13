#! /usr/bin/env python3
from dynamic_reconfigure.server import Server
from bitbots_vision_tools.cfg import ColorTestConfig
from bitbots_vision.vision_modules import color
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import rospkg
import cv2
import os.path


class ColorTest:

    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('bitbots_vision_tools')

        # ROS-Stuff:
        rospy.init_node('bitbots_vision_color_test')

        self.bridge = CvBridge()

        self.config = {}

        # Register publisher for 'color_tester_hsv_mask_image'-messages
        self.pub_hsv_mask_image = rospy.Publisher(
            'color_tester_hsv_mask',
            Image)

        srv = Server(ColorTestConfig, self._dynamic_reconfigure_callback)

        rospy.spin()

    def _image_callback(self, image_msg):
        self.handle_image(image_msg)

    def handle_image(self, image_msg):
        # converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # mask (and publish) image
        self.color_detector.get_mask_image(image)

    def _dynamic_reconfigure_callback(self, config, level):
        self.color_detector = color.HsvSpaceColorDetector(config, "color_test", self.pub_hsv_mask_image)

        # subscriber of camera image:
        if 'img_msg_topic' not in self.config or \
            self.config['img_msg_topic'] != config['img_msg_topic'] or \
            'img_queue_size' not in self.config or \
            self.config['img_queue_size'] != config['img_queue_size']:
            if hasattr(self, 'image_sub'):
                self.image_sub.unregister()
            self.image_sub = rospy.Subscriber(
                config['img_msg_topic'],
                Image,
                self._image_callback,
                queue_size=config['img_queue_size'],
                tcp_nodelay=True,
                buff_size=60000000)
            # https://github.com/ros/ros_comm/issues/536

        self.config = config
        return config

if __name__ == "__main__":
    ColorTest()
