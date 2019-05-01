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

        self.debug_printer = debug.DebugPrinter(
            debug_classes=debug.DebugPrinter.generate_debug_class_list_from_string(
                rospy.get_param('debug_printer_classes')))
        self.use_pixel_list = rospy.get_param('use_PixelListColorDetector')
        # default configuration
        if self.use_pixel_list:
            # TODO: fix PixelListColorDetector (vision config needed)
            self.color_detector = color.PixelListColorDetector(
                self.debug_printer,
                self.package_path + rospy.get_param('path'))
        else:
            self.color_detector = color.HsvSpaceColorDetector(
                self.debug_printer,
                rospy.get_param('lower_values'),
                rospy.get_param('upper_values'))


        srv = Server(ColorTestConfig, self._dynamic_reconfigure_callback)
        # subscriber:
        self.bridge = CvBridge()
        rospy.Subscriber(rospy.get_param('img_msg_topic'),
                         Image,
                         self._image_callback,
                         queue_size=rospy.get_param('img_queue_size'),
                         tcp_nodelay=True,
                         buff_size=60000000)

        # Register publisher for 'color_tester_mask_image'-messages
        # TODO: add topic parameter
        self.imagepublisher = rospy.Publisher(
            'color_tester_mask_image',
            Image,
            queue_size=1)

        rospy.spin()

    def _image_callback(self, img):
        self.handle_image(img)

    def handle_image(self, image_msg):
        # converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # mask image
        mask_img = self.color_detector.mask_image(image)

        # output masked image
        #cv2.imshow('Debug Image', mask_img)
        #cv2.waitKey(1)

        self.imagepublisher.publish(self.bridge.cv2_to_imgmsg(mask_img, '8UC1'))

    def _dynamic_reconfigure_callback(self, config, level):
        self.debug_mode = config['debug_on']
        # exchange color_detector between PixelListColorDetector and HsvSpaceColorDetector
        self.use_pixel_list = config['use_pixel_list']
        if self.debug_mode:
            print('use_pixel_list = ' + str(self.use_pixel_list))
        if self.use_pixel_list:
            # TODO: fix PixelListColorDetector
            # use PixelListColorDetector
            if os.path.isfile(self.package_path +
                              config['pixel_list_path']):
                self.color_detector = color.PixelListColorDetector(self.package_path +
                                                                   config['pixel_list_path'])
                if self.debug_mode:
                    print('color_detector is instance of PixelListColorDetector('
                          + self.package_path
                          + config['pixel_list_path'])
            else:
                print('FILE "'
                      + self.package_path + config['pixel_list_path'] + '" does NOT EXIST')
        else:
            # use HsvSpaceColorDetector
            self.color_detector = color.HsvSpaceColorDetector(
                self.debug_printer,
                (config['h_min'], config['s_min'], config['v_min']),
                (config['h_max'], config['s_max'], config['v_max']))
            if self.debug_mode:
                print('color_detector is instance of HsvSpaceColorDetector('
                      + str((config['h_min'], config['s_min'], config['v_min']))
                      + ', '
                      + str((config['h_max'], config['s_max'], config['v_max'])))
        return config

if __name__ == "__main__":
    ColorTest()