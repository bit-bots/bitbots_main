import numpy as np
import tf2_ros as tf2
from bitbots_blackboard.capsules.game_status_capsule import GameStatusCapsule
from bitbots_localization.srv import ResetFilter, SetPaused
from bitbots_utils.transforms import quat2euler, xyzw2wxyz
from bitbots_utils.utils import get_parameters_from_other_node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
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

        # IMU
        self.accel = np.array([0.0, 0.0, 0.0])
        self.imu_orientation = Quaternion(w=1.0)

        # Falling odometry / imu interpolation during falls
        self.imu_yaw_before_fall: float = 0.0

        # Picked up
        self.pickup_accel_buffer = []
        self.pickup_accel_buffer_long = []

        # Last init action
        self.last_init_action_type = False
        self.last_init_odom_transform: TransformStamped | None = None

    def _callback_pose(self, msg: PoseWithCovarianceStamped):
        self.robot_pose = msg

    def _callback_robot_control_state(self, msg: RobotControlState):
        self.robot_control_state = msg.state

        # Reset pickup buffer if we fall down
        if self.robot_control_state in [
            RobotControlState.FALLEN,
            RobotControlState.FALLING,
            RobotControlState.GETTING_UP,
        ]:
            self.pickup_accel_buffer = []
            self.pickup_accel_buffer_long = []

    def _callback_imu(self, msg: Imu):
        self.accel = numpify(msg.linear_acceleration)
        self.imu_orientation = msg.orientation

        if self.robot_control_state is not None and self.robot_control_state not in [
            RobotControlState.FALLEN,
            RobotControlState.FALLING,
            RobotControlState.GETTING_UP,
        ]:
            self.pickup_accel_buffer.append(self.accel)
            self.pickup_accel_buffer_long.append(self.accel)
            if len(self.pickup_accel_buffer) > 200:
                self.pickup_accel_buffer.pop(0)
            if len(self.pickup_accel_buffer_long) > 10000:
                self.pickup_accel_buffer_long.pop(0)

    def picked_up(self) -> bool:
        """Naive check if the robot is picked up. Only works if the robot is standing still."""
        if len(self.pickup_accel_buffer) == 0:
            return False
        buffer = np.array(self.pickup_accel_buffer)
        mean = np.mean(buffer[..., 2])
        buffer_long = np.array(self.pickup_accel_buffer_long)
        mean_long = np.mean(buffer_long[..., 2])
        absolute_diff = abs(mean_long - mean)

        return absolute_diff > 1.0

    def get_imu_yaw(self) -> float:
        """Returns the current yaw of the IMU (this is not an absolute measurement!!! It drifts over time!)"""

        return quat2euler(xyzw2wxyz(numpify(self.imu_orientation)), axes="szxy")[2]

    def get_localization_yaw(self) -> float:
        """Returns the current yaw of the robot according to the localization. 0 is the x-axis of the field."""
        if self.robot_pose is None:
            return 0.0
        return quat2euler(xyzw2wxyz(numpify(self.robot_pose.pose.pose.orientation)), axes="szxy")[2]
