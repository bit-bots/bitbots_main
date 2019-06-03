#! /usr/bin/env python2
import rospy
import rospkg
import actionlib
import math
import cPickle as pickle
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from humanoid_league_msgs.msg import VisualCompassRotation
from bitbots_visual_compass.cfg import VisualCompassConfig
from worker import VisualCompass


# TODO: rosdep
# TODO: update docs in action
# TODO: dump keypoints of ground truth in pickle file
# TODO: launch file for set ground truth

class VisualCompassStartup():
    # type: () -> None
    """
    TODO docs
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
        self.package_path = rospack.get_path('bitbots_visual_compass')

        rospy.init_node('bitbots_visual_compass_startup')
        rospy.loginfo('Initializing visual compass startup')

        self.bridge = CvBridge()

        self.config = {}
        self.compass = None

        # Register publisher of 'visual_compass'-messages
        self.pub_compass = rospy.Publisher(
            'visual_compass',
            VisualCompassRotation,
            queue_size=1)

        # Register VisualCompassConfig server for dynamic reconfigure and set callback
        Server(VisualCompassConfig, self.dynamic_reconfigure_callback)

        rospy.spin()

    def dynamic_reconfigure_callback(self, config, level):
        # type: (dict, TODO) -> None
        """
        TODO docs
        """
        if self.changed_config_param(config, 'ground_truth_file_name'):
            self.is_ground_truth_set = False

        self.compass = VisualCompass(config)
        self.compass.set_ground_truth_keypoints(self.load_ground_truth(config['ground_truth_file_name']))

        # Subscribe to Image-message
        if self.changed_config_param(config, 'ROS_handler_img_msg_topic'):
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

        return self.config

    def image_callback(self, image_msg):
        # type: (Image) -> None
        """
        TODO docs
        """
        # Drops old images
        # TODO: fix
        # image_age = rospy.get_rostime() - image_msg.header.stamp 
        # if image_age.to_sec() > 0.1:
        #     print("Visual Compass: Dropped Image-message")  # TODO debug printer
        #     return

        self.handle_image(image_msg)

    def handle_image(self, image_msg):
        # type: (Image) -> None
        """
        TODO docs
        """
        # Set image
        # TODO: Set y-axis orientation of IMU
        self.compass.process_image(self.bridge.imgmsg_to_cv2(image_msg, 'bgr8'))

        # Get angle and certainty from compass
        result = self.compass.get_side()

        # Publishes the 'visual_compass'-message
        self.publish_rotation(image_msg.header.frame_id, image_msg.header.stamp, result[0], result[1])
    
    def publish_rotation(self, header_frame_id, header_stamp, orientation, confidence):
        # type: (TODO, TODO, float, float) -> None
        """
        TODO docs
        """
        msg = VisualCompassRotation()

        # Create VisualCompassRotation-message
        msg.header.frame_id = header_frame_id
        msg.header.stamp = header_stamp

        msg.orientation = orientation
        msg.confidence = confidence

        # Publish VisualCompassMsg-message
        self.pub_compass.publish(msg)

    def load_ground_truth(self, ground_truth_file_name):
        # type: (str) -> ([], [])
        """
        TODO docs
        """
        # generate file path
        file_path = self.package_path + ground_truth_file_name
        # load keypoints of pickle file
        ground_truth_keypoints = pickle.load(open( file_path, "rb" ))
        rospy.loginfo('Loaded keypoint file at: %(path)s' % {'path': file_path})
        return ground_truth_keypoints

    def changed_config_param(self, config, param_name):
        # type: (dict, str) -> bool
        """
        TODO
        """
        return param_name not in self.config or config[param_name] != self.config[param_name]

if __name__ == '__main__':
    VisualCompassStartup()
