#! /usr/bin/env python2
import rospy
import rospkg
import actionlib
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server
from humanoid_league_msgs.msg import VisualCompassRotation
from bitbots_visual_compass.cfg import VisualCompassConfig
from bitbots_msgs.msg import VisualCompassSetGroundTruthAction
from worker import VisualCompass
import tf2_ros as tf2
from tf2_geometry_msgs import PoseStamped
from tf.transformations import euler_from_quaternion


class VisualCompassROSHandler():
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
        # self.package_path = rospack.get_path('bitbots_visual_compass')

        rospy.init_node('bitbots_visual_compass')
        rospy.loginfo('Initializing visual compass')

        self.bridge = CvBridge()

        self.config = {}
        self.image_dict = {}
        self.compass = None

        self.base_frame = 'base_footprint'
        self.camera_frame = 'camera_optical_frame'
        self.tf_buffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.tf_buffer)

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
        self.compass = VisualCompass(config)

        if self.changed_config_param(config, 'compass_multiple_sample_count') or \
            self.changed_config_param(config, 'compass_type') or \
            self.changed_config_param(config, 'compass_matcher'):

            self.sample_count = 0
            self.ready_for_loop = False

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

        self.check_sample_count()

        return self.config

    def set_truth_callback(self, goal):

        if self.image_dict:
            # TODO: check timestamps

            orientation = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, goal.header.stamp).transform.rotation
            yaw_angle = euler_from_quaternion(( orientation.x, 
                                                orientation.y, 
                                                orientation.z, 
                                                orientation.w))[2] + 0.5 * math.pi

            self.compass.set_truth(yaw_angle, self.image_dict['image'])
            self.actionServer.set_succeeded()
            self.sample_count += 1
            self.check_sample_count()

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

        # Converting the ROS image message to CV2-image
        self.image_dict['image'] = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        self.image_dict['header_frame_id'] = image_msg.header.frame_id
        self.image_dict['header_stamp'] = image_msg.header.stamp

        if self.ready_for_loop:
            self.loop_over_image(self.image_dict)

    def loop_over_image(self, image_dict):
        # type: (dict) -> None
        """
        TODO docs
        """
        # Set image
        # TODO: Set y-axis orientation of IMU
        self.compass.process_image(image_dict['image'])

        # Get angle and certainty from compass
        result = self.compass.get_side()

        # Publishes the 'visual_compass'-message
        self.publish_rotation(image_dict['header_frame_id'], image_dict['header_stamp'], result[0], result[1])
    
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

    def check_sample_count(self):
        # type: () -> None
        """
        TODO docs
        """
        config_sample_count = self.config['compass_multiple_sample_count']
        if self.sample_count != config_sample_count:
            rospy.loginfo('Visual compass: %(var)d of %(config)d sample images set. More sample images are needed to start processing loop.' %
                            {'var': self.sample_count, 'config': config_sample_count})
            self.ready_for_loop = False
        else:
            if not(self.ready_for_loop):
                rospy.loginfo('Visual compass: Initiating finished. Now Ready for processing loop.')
            self.ready_for_loop = True
            
    def changed_config_param(self, config, param_name):
        # type: (dict, str) -> bool
        """
        TODO
        """
        return param_name not in self.config or config[param_name] != self.config[param_name]

if __name__ == '__main__':
    VisualCompassROSHandler()
