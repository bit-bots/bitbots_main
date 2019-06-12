#! /usr/bin/env python2
from dynamic_reconfigure.server import Server
from bitbots_vision_tools.cfg import ColorTestConfig
from bitbots_vision.vision_modules import color, debug
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

        srv = Server(ColorTestConfig, self._dynamic_reconfigure_callback)

        rospy.spin()

    def _image_callback(self, image_msg):
        self.handle_image(image_msg)

    def handle_image(self, image_msg):
        # converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # mask image
        mask_img = self.color_detector.mask_image(image)

        self.imagepublisher.publish(self.bridge.cv2_to_imgmsg(mask_img, '8UC1'))

    def _dynamic_reconfigure_callback(self, config, level):
        self.debug_printer = debug.DebugPrinter(
            debug_classes=debug.DebugPrinter.generate_debug_class_list_from_string(
                config['debug_printer_classes']))

        self.color_detector = color.HsvSpaceColorDetector(
            self.debug_printer,
            (config['lower_values_h'], config['lower_values_s'], config['lower_values_v']),
            (config['upper_values_h'], config['upper_values_s'], config['upper_values_v']))

        # Register publisher for 'color_tester_mask_image'-messages
        if 'color_tester_mask_image' not in self.config or \
                self.config['color_tester_mask_image'] != config['color_tester_mask_image']:
            self.imagepublisher = rospy.Publisher(
                'color_tester_mask_image',
                Image,
                queue_size=1)

        # subscriber of camera image:
        if 'img_msg_topic' not in self.config or \
            self.config['img_msg_topic'] != config['img_msg_topic'] or \
            'ROS_img_queue_size' not in self.config or \
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
