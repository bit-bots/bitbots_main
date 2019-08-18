#! /usr/bin/env python2
from os import path
import socket
import rospy
import rospkg
import math
import cv2
import pickle
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from dynamic_reconfigure.server import Server
from bitbots_visual_compass.cfg import VisualCompassConfig
from worker import VisualCompass
import tf2_ros as tf2
from tf2_geometry_msgs import PoseStamped
from humanoid_league_msgs.msg import HeadMode
from tf.transformations import euler_from_quaternion
from key_point_converter import KeyPointConverter
from datetime import datetime

# TODO: rosdep, u.a. motion etc...
# TODO: set head mode
# TODO: check published pose
# TODO: fix drop old images (also in startup)

class VisualCompassSetup():
    # type: () -> None
    """
    TODO docs

    Subscribes to raw image
    This sets the head behavior to a special head mode, where it scans for image features above the fieldboundary.
    The head behavior sends a trigger message, if it reaches predefined points. If this node gets trigged the current image features are saved in our feature map.
    Afterwards the map is saved on the robot and the robot handler can download it. Than hes is able to share it with the other robots.
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

        rospy.init_node('bitbots_visual_compass_setup')
        rospy.loginfo('Initializing visual compass setup')

        self.bridge = CvBridge()

        self.config = {}
        self.image_msg = None
        self.compass = None
        self.hostname = socket.gethostname()

        # TODO: docs
        self.base_frame = 'base_footprint'
        self.camera_frame = 'camera_optical_frame'
        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(50))
        self.listener = tf2.TransformListener(self.tf_buffer)

        self.pub_head_mode = rospy.Publisher(
            'head_mode',
            HeadMode,
            queue_size=1)

        # Register VisualCompassConfig server for dynamic reconfigure and set callback
        Server(VisualCompassConfig, self.dynamic_reconfigure_callback)

        rospy.logwarn("------------------------------------------------")
        rospy.logwarn("||                 WARNING                    ||")
        rospy.logwarn("||Please remove the LAN cable from the Robot, ||")
        rospy.logwarn("||after pressing 'YES' you have 10 Seconds    ||")
        rospy.logwarn("||until the head moves OVER the LAN port!!!   ||")
        rospy.logwarn("------------------------------------------------\n\n")

        try:
            input = raw_input
        except NameError:
            pass

        accept = input("Do you REALLY want to start? (YES/n)")

        if accept == "YES":
            rospy.logwarn("REMOVE THE LAN CABLE NOW!!!!!")

            rospy.sleep(10)

            head_mode = HeadMode()
            head_mode.headMode = 10
            self.pub_head_mode.publish(head_mode)
            rospy.loginfo("Head mode has been set!")

            rospy.spin()
        else:
            rospy.signal_shutdown("You aborted the process! Shuting down correctly.")

    def dynamic_reconfigure_callback(self, config, level):
        # type: (dict, TODO) -> None
        """
        TODO docs
        """
        self.compass = VisualCompass(config)

        if self.changed_config_param(config, 'compass_type') or \
            self.changed_config_param(config, 'compass_matcher') or \
            self.changed_config_param(config, 'compass_multiple_map_image_count'):

            self.feature_map_images_count = 0
            self.processed_set_all_feature_map_images = False

            rospy.loginfo('Loaded configuration: compass type: %(type)s | matcher type: %(matcher)s | map images: %(feature_map_count)d' % {
                    'type': config['compass_type'],
                    'matcher': config['compass_matcher'],
                    'feature_map_count': config['compass_multiple_map_image_count']})

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

        # Register message server to call set truth callback
        if self.changed_config_param(config, 'feature_map_trigger_topic') or \
            self.changed_config_param(config, 'feature_map_trigger_queue_size'):
            if hasattr(self, 'sub_trigger_set_feature_map'):
                self.sub_image_msg.unregister()
            self.sub_trigger_set_feature_map = rospy.Subscriber(
                config['feature_map_trigger_topic'],
                Header,
                self.set_truth_callback,
                queue_size=config['feature_map_trigger_queue_size'])

        self.config = config

        self.check_image_count()

        return self.config

    def set_truth_callback(self, request):
        if self.image_msg:
            # TODO: check timestamps

            orientation = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, self.image_msg.header.stamp, timeout=rospy.Duration(0.5)).transform.rotation
            yaw_angle = (euler_from_quaternion((
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w))[2] + 0.5 * math.pi) % (2 * math.pi)

            image = self.bridge.imgmsg_to_cv2(self.image_msg, 'bgr8')

            self.compass.set_truth(yaw_angle, image)
            self.feature_map_images_count += 1
            self.check_image_count()

        else:
            rospy.logwarn('No image received yet.')

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

        self.image_msg = image_msg

    def check_image_count(self):
        # type: () -> None
        """
        TODO docs
        """
        config_feature_map_images_count = self.config['compass_multiple_map_image_count']
        if self.feature_map_images_count != config_feature_map_images_count:
            rospy.loginfo('Visual compass: %(var)d of %(config)d map images set. More images are needed.' %
                            {'var': self.feature_map_images_count, 'config': config_feature_map_images_count})
            self.processed_set_all_feature_map_images = False
        else:
            if not(self.processed_set_all_feature_map_images):
                rospy.loginfo('Visual compass: All map images have been processed.')
                self.save_feature_map(self.config['feature_map_file_path'])
            self.processed_set_all_feature_map_images = True

    def save_feature_map(self, feature_map_file_path):
        # type (str) -> None
        """
        TODO docs
        """
        converter = KeyPointConverter()

        # get keypoints and mean feature count per image
        features = self.compass.get_feature_map()

        mean_feature_count = self.compass.get_mean_feature_count()

        # convert keypoints to basic values
        keypoints = features[0]
        keypoint_values = [converter.key_point2values(kp) for kp in keypoints]

        descriptors = features[1]

        meta = {
            'field': self.config['feature_map_field'],
            'date': datetime.now(),
            'device': self.hostname,
            'compass_type': self.config['compass_type'],
            'compass_matcher': self.config['compass_matcher'],
            'compass_multiple_map_image_count': self.config['compass_multiple_map_image_count'],
            'keypoint_count': len(keypoint_values),
            'descriptor_count': len(descriptors),
            'mean_feature_count': mean_feature_count,
        }

        dump_features = {
            'keypoint_values': keypoint_values,
            'descriptors': descriptors,
            'meta': meta}

        # generate file path
        file_path = self.package_path + feature_map_file_path
        # warn, if file does exist allready
        if path.isfile(file_path):
            rospy.logwarn('Map file at: %(path)s does ALLREADY EXIST. This will be overwritten.' % {'path': file_path})
        # save keypoints in pickle file
        with open(file_path, 'wb') as f:
            pickle.dump(dump_features, f)
        info_str = "\n\t-----------------------------------------------------------------------------------------------------------------\n" + \
        "\tSaved map file at: %(path)s\n" % {'path': file_path} + \
        "\tRUN the following command on your system (NOT THE ROBOT) to save the map file in your current directory:\n" + \
        "\n\tscp bitbots@%(host)s:%(path)s .\n" % {'path': file_path, 'host': self.hostname} + \
        "\t-----------------------------------------------------------------------------------------------------------------"
        rospy.loginfo(info_str)

        # shutdown setup process
        rospy.signal_shutdown('Visual compass setup finished cleanly.')

    def changed_config_param(self, config, param_name):
        # type: (dict, str) -> bool
        """
        TODO
        """
        return param_name not in self.config or config[param_name] != self.config[param_name]

if __name__ == '__main__':
    VisualCompassSetup()
