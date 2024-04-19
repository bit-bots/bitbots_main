import numpy as np
import tf2_ros as tf2
from bitbots_blackboard.capsules.game_status_capsule import GameStatusCapsule
from bitbots_localization.srv import ResetFilter, SetPaused
from bitbots_utils.utils import get_parameters_from_other_node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from ros2_numpy import numpify
from sensor_msgs.msg import Imu
from tf2_geometry_msgs import TransformStamped

from bitbots_msgs.msg import RobotControlState


class LocalizationBlackboard:
    def __init__(self, node: Node):
        self.node = node

        self.initialized = False
        self.use_sim_time = self.node.get_parameter("use_sim_time").value

        # we only need tf in simulation. don't use it otherwise to safe performance
        if self.use_sim_time:
            self.tf_buffer = tf2.Buffer(cache_time=Duration(seconds=10))
            self.tf_listener = tf2.TransformListener(self.tf_buffer, node)

        # Get names of relevant frames
        self.odom_frame: str = node.get_parameter("odom_frame").value
        self.base_footprint_frame: str = node.get_parameter("base_footprint_frame").value

        # Get the length of the field
        self.field_length = get_parameters_from_other_node(self.node, "parameter_blackboard", ["field_length"])[
            "field_length"
        ]

        # Service clients
        self.reset_filter_proxy = node.create_client(ResetFilter, "reset_localization")
        self.stop_filter_proxy = node.create_client(SetPaused, "pause_localization")

        # The current pose of the robot
        self.robot_pose: PoseWithCovarianceStamped | None = None

        # GameState
        self.gamestate = GameStatusCapsule(node)

        # Robot Control State
        self.robot_control_state: RobotControlState | None = None

        # Was the last robot state one of the get up states?
        self.last_state_get_up = False

        # Picked up
        self.accel = np.array([0, 0, 0])
        self.accel_buffer = []

        # Last init action
        self.last_init_action_type = False
        self.last_init_odom_transform: TransformStamped | None = None

    def _callback_pose(self, msg: PoseWithCovarianceStamped):
        self.robot_pose = msg

    def _callback_robot_control_state(self, msg: RobotControlState):
        self.robot_control_state = msg.state

    def _callback_imu(self, msg: Imu):
        self.accel = numpify(msg.linear_acceleration)
        self.accel_buffer.append(self.accel)
        if len(self.accel_buffer) > 500:
            self.accel_buffer.pop(0)

    def picked_up(self) -> bool:
        """Naive check if the robot is picked up. Only works if the robot is standing still."""
        if len(self.accel_buffer) == 0:
            return False
        buffer = np.array(self.accel_buffer)
        mean = np.mean(buffer[..., 2])
        g = 9.81
        absolute_diff = abs(g - mean)

        return absolute_diff > 0.9
