#! /usr/bin/env python2


from vision_modules import color
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import rospkg
import cv2

class ColorTest:

    def __init__(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('bitbots_vision')

        # decide using PixelListColorDetector or HsvSpaceColorDetector
        if rospy.get_param('color_test_params/color_detector_implementation/use_PixelListColorDetector'):
            self.color_detector = color.PixelListColorDetector(
                package_path +
                rospy.get_param('color_test_params/field_color_detector/path'))
        else:
            self.color_detector = color.HsvSpaceColorDetector(
                rospy.get_param('color_test_params/white_color_detector/lower_values'),
                rospy.get_param('color_test_params/white_color_detector/upper_values'))

        # ROS-Stuff:
        rospy.init_node('bitbots_vision_color_test')

        # subscriber:
        self.bridge = CvBridge()
        rospy.Subscriber(rospy.get_param('color_test_params/ROS/img_msg_topic'),
                         Image,
                         self._image_callback,
                         queue_size=rospy.get_param(
                             'color_test_params/ROS/img_queue_size'))

        rospy.spin()

    def _image_callback(self, img):
        self.handle_image(img)

    def handle_image(self, image_msg):
        # converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # mask image
        mask_img = self.color_detector.mask_image(image)

        # output masked image
        cv2.imshow('Debug Image', mask_img)
        cv2.waitKey(1)

if __name__ == "__main__":
    ColorTest()