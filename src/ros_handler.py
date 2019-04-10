#! /usr/bin/env python2

import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from bitbots_msgs.msg import VisualCompassMsg
from bitbots_vision.vision_modules import debug
from visual_compass import VisualCompassConfig
from worker import VisualCompass

# TODO adapt import paths
# TODO define message and trigger
# TODO use compass.set_truth

class VisualCompassROSHandler():
# type: () -> None
"""
TODO docs
Subscribes to 'vision_config'-message
Subscribes to raw image

Trigger: 'trigger_visual_compass'-trigger
    Gets triggered e.i. while looking at a goal side
    Returns side

Publish: 'visual_compass'-messages
    Returns side
"""
    def __init__(self):
        # type: () -> None
        """
        Initiate VisualCompassHandler

        return: None
        """
        # Init ROS package
        rospack = rospkg.RosPack()
        # self.package_path = rospack.get_path('bitbots_visual_compass')

        rospy.init_node('bitbots_visual_compass')
        rospy.loginfo('Initializing visual compass')

        self.bridge = CvBridge()

        self.config = {}
        self.compass = None

        # Register publisher of 'visual_compass'-messages
        self.pub_compass = rospy.Publisher(
            'visual_compass',
            VisualCompassMsg,
            queue_size=1)

        # Register VisionConfig server (dynamic reconfigure) and set callback
        srv = Server(VisualCompassConfig, self._dynamic_reconfigure_callback)

        rospy.spin

    def _dynamic_reconfigure_callback(self, config, level):
        # type: (dict, TODO) -> None
        """
        TODO docs
        """
        self.debug_printer = debug.DebugPrinter(
            debug_classes=debug.DebugPrinter.generate_debug_class_list_from_string(
                config['visual_compass_debug_printer_classes']))

        if self.compass == None:
            # Create compass
            self.compass = VisualCompass(config)
        else:
            # Set config
            self.compass.config_callback(config)

        # Subscribe to Image-message
        if 'ROS_img_msg_topic' not in self.config or \
                self.config['ROS_img_msg_topic'] != config['ROS_img_msg_topic']:
            if hasattr(self, 'sub_image_msg'):
                self.sub_image_msg.unregister()
            self.sub_image_msg = rospy.Subscriber(
                config['ROS_img_msg_topic'],
                Image,
                self.image_callback,
                queue_size=config['ROS_img_queue_size'],
                tcp_nodelay=True,
                buff_size=60000000)
            # https://github.com/ros/ros_comm/issues/536

        self.config = config

    def image_callback(self, image_msg):
        # type: (Image) -> None
        """
        TODO docs
        """
        # Drops old images
        image_age = rospy.get_rostime() - image_msg.header.stamp 
        if image_age.to_sec() > 0.1:
            self.debug_printer.info('Visual Compass: Dropped Image-message', 'image')
            return

        self.handle_image(image_msg)

    def handle_image(self, image_msg):
        # type: (Image) -> None
        """
        TODO docs
        """
        # Converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # Set image
        self.compass.image_callback(image)

        # Get angle and certainty from compass
        result = self.compass.get_side()

        # Publishes the 'visual_compass'-message
        self.publish(image_msg, result[0], result[1])
    
    def publish(self, image_msg, angle, certainty):
        # type: (Image, float, float) -> None
        """
        TODO docs
        """
        msg = VisualCompassMsg()

        # Create VisualCompassMsg-message
        msg.header.frame_id = image_msg.header.frame_id
        msg.header.stamp = image_msg.header.stamp

        msg.angle = angle
        msg.certainty = certainty

        # Publish VisualCompassMsg-message
        self.pub_compass.publish(msg)


if __name__ == '__main__':
    VisualCompassROSHandler()