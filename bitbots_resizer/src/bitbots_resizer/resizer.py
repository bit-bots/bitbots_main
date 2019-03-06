#! /usr/bin/env python2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import rospkg
import cv2


class Resizer:

    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('bitbots_resizer')

        rospy.init_node('bitbots_resizer')
        rospy.loginfo('Initializing resizer...')

        self.bridge = CvBridge()

        input_topic = rospy.get_param("~input_topic")

        output_topic = rospy.get_param("~output_topic") 

        self.pub_resized_image = rospy.Publisher(
            output_topic,
            Image,
            queue_size=1,
        )

        self.image_sub = rospy.Subscriber(input_topic,
            Image,
            self._image_callback,
            tcp_nodelay=True,
            buff_size=60000000)

        rospy.spin()

    def _image_callback(self, img):
        # converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        # resizing the image
        height = rospy.get_param("~height")
        width = rospy.get_param("~width")
        resized_image = cv2.resize(image, (width, height))
        # publishes processed image
        self.pub_resized_image.publish(self.bridge.cv2_to_imgmsg(resized_image, 'bgr8'))

if __name__ == '__main__':
    Resizer()
