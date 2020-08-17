#! /usr/bin/env python3
from os import path
import rospy
import rospkg
import actionlib
import math
from cv2 import KeyPoint
import cPickle as pickle
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from tf_conversions.transformations import quaternion_from_euler
from humanoid_league_msgs.msg import GameState, PoseWithCertaintyStamped
from bitbots_visual_compass.cfg import VisualCompassConfig
from worker import VisualCompass
from key_point_converter import KeyPointConverter
from visual_compass_filter import VisualCompassFilter


class VisualCompassStartup():
    # type: () -> None
    """
    This node runs the visual compass in the game. It receives the current image and executes the visual compas worker.
    In addition it loads the map describing the field background. In the end it executes some filters and publishes the results.

    Subscribes to raw image

    Publish: 'visual_compass'-messages
        Returns angle, confidence
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
        self.orientation_offset = 0  # Orientation changes about PI in the second game half

        self.filter = None

        self.lastTimestamp = rospy.Time.now()

        # Register publisher of 'visual_compass'-messages
        self.pub_compass = rospy.Publisher(
            'visual_compass',
            PoseWithCertaintyStamped,
            queue_size=1)

        # Register VisualCompassConfig server for dynamic reconfigure and set callback
        Server(VisualCompassConfig, self.dynamic_reconfigure_callback)

        rospy.spin()

    def dynamic_reconfigure_callback(self, config, level):
        # type: (dict, TODO) -> None
        """
        Dynamic reconfigure callback that sets a new configuration
        """
        self.compass = VisualCompass(config)
        feature_map, meta_data = self.load_feature_map(config['feature_map_file_path'])
        self.compass.set_feature_map(feature_map)
        self.compass.set_mean_feature_count(meta_data['mean_feature_count'])

        self.filter = VisualCompassFilter()

        if self.changed_config_param(config, 'feature_map_file_path'):
            self.is_feature_map_set = False

        if self.changed_config_param(config, 'compass_type') or \
            self.changed_config_param(config, 'compass_matcher') or \
            self.changed_config_param(config, 'compass_multiple_map_image_count'):

            rospy.loginfo('Loaded configuration: compass type: %(type)s | matcher type: %(matcher)s | map images: %(feature_map_count)d' % {
                    'type': config['compass_type'],
                    'matcher': config['compass_matcher'],
                    'feature_map_count': config['compass_multiple_map_image_count']})

        # Subscribe to game state
        self.game_state_msg = rospy.Subscriber(
            'gamestate',
            GameState,
            self.gamestate_callback,
            queue_size=1,
            tcp_nodelay=True)

        # Subscribe to Image-message
        if self.changed_config_param(config, 'img_msg_topic') or \
            self.changed_config_param(config, 'img_msg_queue_size'):
            if hasattr(self, 'sub_image_msg'):
                self.sub_image_msg.unregister()
            self.sub_image_msg = rospy.Subscriber(
                config['img_msg_topic'],
                Image,
                self.image_callback,
                queue_size=config['img_msg_queue_size'],
                tcp_nodelay=True,
                buff_size=60000000)
            # https://github.com/ros/ros_comm/issues/536

        self.config = config

        return self.config

    def image_callback(self, image_msg):
        # type: (Image) -> None
        """
        Callback that receives the current image and runs the calculation.
        """
        # Drops old images
        # TODO: fix
        # image_age = rospy.get_rostime() - image_msg.header.stamp
        # if image_age.to_sec() > 0.1:
        #     print("Visual Compass: Dropped Image-message")  # TODO debug printer
        #     return
        now = rospy.Time.now()
        if rospy.Duration(1) < now - self.lastTimestamp:
            self.handle_image(image_msg)
            self.lastTimestamp = rospy.Time.now()

    def handle_image(self, image_msg):
        # type: (Image) -> None
        """
        Runs the visual compass worker and filter.
        """

        image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # Set image
        compass_result_angle, compass_result_confidence = self.compass.process_image(image)

        # Filter results
        result = self.filter.filterMeasurement(compass_result_angle, compass_result_confidence, image_msg.header.stamp)

        # Publishes the 'visual_compass'-message
        self.publish_rotation("base_footprint", image_msg.header.stamp, result[0], result[1])

    def gamestate_callback(self, msg):
        """
        Recives the game state to determin which side is ours.
        """
        if msg.firstHalf:
            self.orientation_offset = 0
        else:
            self.orientation_offset = math.pi

    def publish_rotation(self, header_frame_id, header_stamp, orientation, confidence):
        # type: (TODO, TODO, float, float) -> None
        """
        Builds the ros message and publishes the result.
        """
        msg = PoseWithCertaintyStamped()

        # Create PoseWithCertaintyStamped-message where only the orentation is used
        msg.header.frame_id = header_frame_id
        msg.header.stamp = header_stamp

        msg.pose.pose.pose.orientation = quaternion_from_euler(0,0,(orientation + self.orientation_offset) % (2 * math.pi))  # Orientation changes about PI in the second game half
        msg.pose.confidence = confidence

        # Publish VisualCompassMsg-message
        self.pub_compass.publish(msg)



    def load_feature_map(self, feature_map_file_path):
        # type: (str) -> ([], [])
        """
        Loads the map describing the surrounding field background
        """
        # generate file path
        file_path = self.package_path + feature_map_file_path
        features = ([], [])

        if path.isfile(file_path):
            # load keypoints of pickle file
            with open(file_path, 'rb') as f:
                features = pickle.load(f)
            rospy.loginfo('Loaded map file at: %(path)s' % {'path': file_path})

            keypoint_values = features['keypoint_values']
            descriptors = features['descriptors']
            meta = features['meta']

            # TODO broken
            # self.check_meta_information(meta)

            # convert keypoint values to cv2 Keypoints
            keypoints = [KeyPoint(kp[0], kp[1], kp[2], kp[3], kp[4], kp[5], kp[6]) for kp in keypoint_values]

            return ((keypoints, descriptors), meta)
        else:
            rospy.logerr('NO map file found at: %(path)s' % {'path': file_path})

    def check_meta_information(self, meta):
        # type: (dict) -> None
        """
        Ensures that the loaded map is compatible with the selected compass.
        :param meta: meta information of the map
        """
        rospy.loginfo('The map file was recorded at field %(field)a at date %(date)s on device %(device)' % {
            'field': meta['field'], 'date': meta['date'], 'device': meta['device']})

        if meta['keypoint_count'] != meta['descriptor_count']:
            rospy.logerr('Number of keypoints does not match number of descriptors in map file.')
        elif meta['compass_type'] != self.config['compass_type']:
            rospy.logwarn('Config parameter "compass_type" does not match type in map:\n' + \
                'config: %(config)s | map: %(map)a' % {'config': self.config['compass_type'], 'map': meta['compass_type']})
        elif meta['compass_matcher'] != self.config['compass_matcher']:
            rospy.logwarn('Config parameter "compass_compass" does not match parameter in map:\n' + \
                'config: %(config)s | map: %(map)a' % {'config': self.config['compass_matcher'], 'map': meta['compass_matcher']})
        elif meta['compass_multiple_map_image_count'] != self.config['compass_multiple_map_image_count']:
            rospy.logwarn('Config parameter "compass_multiple_map_image_count" does not match parameter in map:\n' + \
                'config: %(config)s | map: %(gt)a' % {'config': self.config['compass_multiple_map_image_count'], 'gt': meta['compass_multiple_map_image_count']})

    def changed_config_param(self, config, param_name):
        # type: (dict, str) -> bool
        """
        TODO
        """
        return param_name not in self.config or config[param_name] != self.config[param_name]

if __name__ == '__main__':
    VisualCompassStartup()
