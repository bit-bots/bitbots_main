import numpy as np
from bitbots_blackboard.capsules.game_status_capsule import GameStatusCapsule
from bitbots_localization.srv import ResetFilter, SetPaused
from bitbots_utils.utils import get_parameters_from_other_node
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from jaxtyping import Float
from rclpy.node import Node
from rclpy.time import Time
from ros2_numpy import numpify
from sensor_msgs.msg import Imu

from bitbots_msgs.msg import RobotControlState


class LocalizationBlackboard:
    def __init__(self, node: Node):
        self.node = node

        self.initialized = False

        # Get the length of the field
        self.field_length = get_parameters_from_other_node(self.node, "parameter_blackboard", ["field.size.x"])[
            "field.size.x"
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
        self.accel: Float[np.ndarray, "3"] = np.zeros(3)
        self.imu_orientation = Quaternion(w=1.0)

        # Last time we have detected a whistle
        self.last_timestep_whistle_detected: Time | None = None

    def _callback_pose(self, msg: PoseWithCovarianceStamped):
        self.robot_pose = msg

    def _callback_robot_control_state(self, msg: RobotControlState):
        self.robot_control_state = msg.state

    def _callback_imu(self, msg: Imu):
        self.accel = numpify(msg.linear_acceleration)
        self.imu_orientation = msg.orientation

    def whistle_detection_callback(self, msg: TimeMsg) -> None:
        self.last_timestep_whistle_detected = Time.from_msg(msg)
