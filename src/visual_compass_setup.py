#! /usr/bin/env python2
import rospy
import rospkg
import actionlib
import math
import cPickle as pickle
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from bitbots_visual_compass.cfg import VisualCompassConfig
from bitbots_msgs.msg import VisualCompassSetGroundTruthAction
from worker import VisualCompass
import tf2_ros as tf2
from tf2_geometry_msgs import PoseStamped
from tf.transformations import euler_from_quaternion

# TODO: rosdep
# TODO: rename sample to ground_truth_images
# TODO: update docs in action
# TODO: dump keypoints of ground truth in pickle file
# TODO: launch file for set ground truth

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

        self.base_frame = 'base_footprint'
        self.camera_frame = 'camera_optical_frame'
        self.tf_buffer = tf2.Buffer()
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

        if self.changed_config_param(config, 'compass_multiple_sample_count') or \
            self.changed_config_param(config, 'compass_type') or \
            self.changed_config_param(config, 'compass_matcher'):

            self.ground_truth_images_count = 0
            self.processed_set_all_ground_truth_images = False

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

        # Register VisualCompassSetGroundTruthAction server
        if self.changed_config_param(config, 'ROS_handler_action_server_name'):
            self.actionServer = actionlib.SimpleActionServer(config['ROS_handler_action_server_name'],
                                                            VisualCompassSetGroundTruthAction,
                                                            execute_cb=self.set_truth_callback,
                                                            auto_start = False)
            self.actionServer.start()

        self.config = config

        self.check_ground_truth_images_count()

        return self.config

    def set_truth_callback(self, goal):
        if self.image_msg:
            # TODO: check timestamps

            orientation = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, goal.header.stamp).transform.rotation
            yaw_angle = euler_from_quaternion(( orientation.x, 
                                                orientation.y, 
                                                orientation.z, 
                                                orientation.w))[2] + 0.5 * math.pi

            image = self.bridge.imgmsg_to_cv2(self.image_msg, 'bgr8')

            self.compass.set_truth(yaw_angle, image)
            self.actionServer.set_succeeded()
            self.ground_truth_images_count += 1
            self.check_ground_truth_images_count()

        else:
            self.actionServer.set_aborted(text="No image received yet")
            rospy.logwarn('No image received yet')

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
        config_ground_truth_images_count = self.config['compass_multiple_sample_count']
        if self.ground_truth_images_count != config_ground_truth_images_count:
            rospy.loginfo('Visual compass: %(var)d of %(config)d ground truth images set. More images are needed.' %
                            {'var': self.ground_truth_images_count, 'config': config_ground_truth_images_count})
            self.processed_set_all_ground_truth_images = False
        else:
            if not(self.processed_set_all_ground_truth_images):
                rospy.loginfo('Visual compass: All ground truth images have been processed.')
                self.save_ground_truth(self.config['ground_truth_file_name'])
            self.processed_set_all_ground_truth_images = True

    def save_ground_truth(self, ground_truth_file_name):
        # type (str) -> None
        """
        TODO docs
        """
        # get keypoints
        keypoints = self.compass.get_ground_truth_keypoints()
        # generate file path
        file_path = self.package_path + ground_truth_file_name
        # save keypoints in pickle file
        pickle.dump(keypoints, open(file_path, "wb"))

    def changed_config_param(self, config, param_name):
        # type: (dict, str) -> bool
        """
        TODO
        """
        return param_name not in self.config or config[param_name] != self.config[param_name]

if __name__ == '__main__':
    VisualCompassSetup()
