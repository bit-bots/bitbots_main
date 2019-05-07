#! /usr/bin/env python2
import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from humanoid_league_msgs.msg import VisualCompassRotation
from bitbots_visual_compass.cfg import VisualCompassConfig
from worker import VisualCompass

# TODO define message for behavior
# TODO use set truth
# TODO compass type|matcher ENUM in .cfg

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
            VisualCompassRotation,
            queue_size=1)

        # Register VisualCompassConfig server for dynamic reconfigure and set callback
        Server(VisualCompassConfig, self._dynamic_reconfigure_callback)

        rospy.spin()

    def _dynamic_reconfigure_callback(self, config, level):
        # type: (dict, TODO) -> None
        """
        TODO docs
        """
        self.compass = VisualCompass(config)

        if 'compass_multiple_sample_count' not in self.config or \
            config['compass_multiple_sample_count'] != self.vision_config['compass_multiple_sample_count']:
            self.sample_count = 0
            self.ready_for_loop = False # TODO change name

        # Subscribe to Image-message
        if 'ROS_handler_img_msg_topic' not in self.config or \
                self.config['ROS_handler_img_msg_topic'] != config['ROS_handler_img_msg_topic']:
            if hasattr(self, 'sub_image_msg'):
                self.sub_image_msg.unregister()
            self.sub_image_msg = rospy.Subscriber(
                config['ROS_handler_img_msg_topic'],
                Image,
                self.image_callback,
                queue_size=config['ROS_handler_img_queue_size'],
                tcp_nodelay=True,
                buff_size=60000000)
            # https://github.com/ros/ros_comm/issues/536

        self.config = config

        self.update_sample_count()

        return self.config

    def image_callback(self, image_msg):
        # type: (Image) -> None
        """
        TODO docs
        """
        # Drops old images
        image_age = rospy.get_rostime() - image_msg.header.stamp 
        if image_age.to_sec() > 0.1:
            print("Visual Compass: Dropped Image-message")  # TODO debug printer
            return

        if self.ready_for_loop:
            self.handle_image(image_msg)

    def handle_image(self, image_msg):
        # type: (Image) -> None
        """
        TODO docs
        """
        # Converting the ROS image message to CV2-image
        image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # Set image
        self.compass.process_image(image)

        # Get angle and certainty from compass
        result = self.compass.get_side()

        # Publishes the 'visual_compass'-message
        self.publish(image_msg, result[0], result[1])
    
    def publish(self, image_msg, orientation, confidence):
        # type: (Image, float, float) -> None
        """
        TODO docs
        """
        msg = VisualCompassRotation()

        # Create VisualCompassRotation-message
        msg.header.frame_id = image_msg.header.frame_id
        msg.header.stamp = image_msg.header.stamp

        msg.orientation = orientation
        msg.confidence = confidence

        # Publish VisualCompassMsg-message
        self.pub_compass.publish(msg)

    def update_sample_count(self):
        # type: () -> None
        """
        TODO docs
        """
        config_sample_count = self.config['compass_multiple_sample_count']
        if self.sample_count != config_sample_count:
            rospy.logwarn('Visual compass: %(var)d of %(config)d sample images set. More sample images are needed to start processing loop.' %
                            {'var': self.sample_count, 'config': config_sample_count})
            self.ready_for_loop = False
        else:
            if not(self.ready_for_loop):
                rospy.loginfo('Visual compass: Initiating finished. Now Ready for processing loop.')
            self.ready_for_loop = True
            


if __name__ == '__main__':
    VisualCompassROSHandler()