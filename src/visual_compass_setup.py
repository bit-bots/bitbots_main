#! /usr/bin/env python2
from os import path
import socket
import rospy
import rospkg
import math
import cv2
import cPickle as pickle
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from dynamic_reconfigure.server import Server
from bitbots_visual_compass.cfg import VisualCompassConfig
from worker import VisualCompass
import tf2_ros as tf2
from tf2_geometry_msgs import PoseStamped
from tf.transformations import euler_from_quaternion
from key_point_converter import KeyPointConverter
from datetime import datetime

# TODO: rosdep
# TODO: launch headbehavior etc
# TODO: check published pose 
# TODO: fix drop old images (also in startup)

class VisualCompassSetup():
    # type: () -> None
    """
    TODO docs
    Subscribes to raw image

    Trigger: 'trigger_visual_compass'-trigger
        Gets triggered e.i. while looking at a goal side
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
        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(5))
        self.listener = tf2.TransformListener(self.tf_buffer)

        # Register VisualCompassConfig server for dynamic reconfigure and set callback
        Server(VisualCompassConfig, self.dynamic_reconfigure_callback)

        rospy.spin()

    def dynamic_reconfigure_callback(self, config, level):
        # type: (dict, TODO) -> None
        """
        TODO docs
        """
        self.compass = VisualCompass(config)

        if self.changed_config_param(config, 'compass_type') or \
            self.changed_config_param(config, 'compass_matcher') or \
            self.changed_config_param(config, 'compass_multiple_ground_truth_images_count'):

            self.ground_truth_images_count = 0
            self.processed_set_all_ground_truth_images = False

            rospy.loginfo('Loaded configuration: compass type: %(type)s | matcher type: %(matcher)s | ground truth images: %(ground_truth_count)d' % {
                    'type': config['compass_type'],
                    'matcher': config['compass_matcher'],
                    'ground_truth_count': config['compass_multiple_ground_truth_count']})

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
        if self.changed_config_param(config, 'ground_truth_trigger_topic') or \
            self.changed_config_param(config, 'ground_truth_trigger_queue_size'):
            if hasattr(self, 'sub_trigger_set_ground_truth'):
                self.sub_image_msg.unregister()
            self.sub_trigger_set_ground_truth = rospy.Subscriber(
                config['ground_truth_trigger_topic'],
                Header,
                self.set_truth_callback,
                queue_size=config['ground_truth_trigger_queue_size'])

        self.config = config

        self.check_ground_truth_images_count()

        return self.config

    def set_truth_callback(self, request):
        if self.image_msg:
            # TODO: check timestamps

            orientation = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, self.image_msg.header.stamp, timeout=rospy.Duration(0.5)).transform.rotation
            yaw_angle = euler_from_quaternion((
                orientation.x, 
                orientation.y, 
                orientation.z, 
                orientation.w))[2] + 0.5 * math.pi

            image = self.bridge.imgmsg_to_cv2(self.image_msg, 'bgr8')

            self.compass.set_truth(yaw_angle, image)
            self.ground_truth_images_count += 1
            self.check_ground_truth_images_count()

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

    def check_ground_truth_images_count(self):
        # type: () -> None
        """
        TODO docs
        """
        config_ground_truth_images_count = self.config['compass_multiple_ground_truth_images_count']
        if self.ground_truth_images_count != config_ground_truth_images_count:
            rospy.loginfo('Visual compass: %(var)d of %(config)d ground truth images set. More images are needed.' %
                            {'var': self.ground_truth_images_count, 'config': config_ground_truth_images_count})
            self.processed_set_all_ground_truth_images = False
        else:
            if not(self.processed_set_all_ground_truth_images):
                rospy.loginfo('Visual compass: All ground truth images have been processed.')
                self.save_ground_truth(self.config['ground_truth_file_path'])
            self.processed_set_all_ground_truth_images = True

    def save_ground_truth(self, ground_truth_file_path):
        # type (str) -> None
        """
        TODO docs
        """
        converter = KeyPointConverter()

        # get keypoints
        features = self.compass.get_ground_truth_features()

        # convert keypoints to basic values
        keypoints = features[0]
        keypoint_values = [converter.key_point2values(kp) for kp in keypoints]

        descriptors = features[1]

        meta = {
            'field': self.config['ground_truth_field'],
            'date': datetime.now(),
            'device': self.hostname,
            'compass_type': self.config['compass_type'],
            'compass_matcher': self.config['compass_matcher'],
            'compass_multiple_ground_truth_images_count' self.config['compass_multiple_ground_truth_images_count'],
            'keypoint_count': len(keypoint_values)
            'descriptor_count': len(descriptors)}

        dump_features = {
            'keypoint_values': keypoint_values, 
            'descriptors': descriptors,
            'meta': meta}

        # generate file path
        file_path = self.package_path + ground_truth_file_path
        # warn, if file does exist allready
        if path.isfile(file_path):
            rospy.logwarn('Ground truth file at: %(path)s does ALLREADY EXIST. This will be overwritten.' % {'path': file_path})
        # save keypoints in pickle file
        with open(file_path, 'wb') as f:
            pickle.dump(dump_features, f)
        info_str = "\n\t-----------------------------------------------------------------------------------------------------------------\n" + \
        "\tSaved ground truth file at: %(path)s\n" % {'path': file_path} + \
        "\tRUN the following command on your system (NOT THE ROBOT) to save the ground truth file in your current directory:\n" + \
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
