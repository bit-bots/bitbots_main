import numpy as np
import tf2_ros as tf2
from bitbots_blackboard.capsules.game_status_capsule import GameStatusCapsule
from rclpy.duration import Duration
from bitbots_localization.srv import ResetFilter, SetPaused
from rclpy.node import Node

class LocalizationBlackboard:

    def __init__(self, node:Node):
        self.node = node

        self.shut_down_request = False
        self.last_initialized = None
        self.initialized = False
        self.use_sim_time = self.node.get_parameter('use_sim_time').value

        # we only need tf in simulation. don't use it otherwise to safe performance
        if self.use_sim_time:
            self.tf_buffer = tf2.Buffer(cache_time=Duration(seconds=10))
            self.tf_listener = tf2.TransformListener(self.tf_buffer, node)
        self.odom_frame = node.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_footprint_frame = node.get_parameter('base_footprint_frame').get_parameter_value().string_value

        self.field_length = node.get_parameter('field_length').get_parameter_value().double_value

        # services
        self.reset_filter_proxy = node.create_client(ResetFilter, 'reset_localization')
        self.stop_filter_proxy = node.create_client(SetPaused, 'pause_localization')

        # Pose
        self.last_pose_update_time = None
        self.poseX = 0
        self.poseY = 0
        self.orientation = np.array([0, 0, 0, 1])
        self.covariance = np.array([])

        #GameState
        self.gamestate = GameStatusCapsule(node)

        #Robot Control State
        self.robot_control_state = None
        self.last_robot_control_state = None

        #Get up
        self.last_state_get_up = False

        #Picked up
        self.last_state_pickup = False

        #Last init action
        self.last_init_action_type = False
        self.last_init_odom_transform = None

    def _callback_pose(self, msg):
        self.last_pose_update_time = msg.header.stamp
        self.poseX = msg.pose.pose.position.x
        self.poseY = msg.pose.pose.position.y
        self.orientation = msg.pose.pose.orientation
        self.covariance = msg.pose.covariance

    def _callback_robot_control_state(self, msg):
        self.robot_control_state = msg.state
