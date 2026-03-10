from geometry_msgs.msg import Twist
from handler.command_handler import CommandHandler
from handler.gravity_handler import GravityHandler
from handler.gyro_handler import GyroHandler
from handler.joints_handler import JointsHandler
from sensor_msgs.msg import Imu, JointState

from bitbots_msgs.msg import JointCommand
from src.rl_node import RLNode


class WalkNode(RLNode):
    def __init__(self, walk_policy_path):
        super().__init__(walk_policy_path)

        # publishers
        self._joint_command_pub = self.create_publisher(JointCommand, "walking_motor_goals", 10)

        # suscribers
        self._imu_sub = self.create_subscription(Imu, "imu/data", self._imu_callback, 10)
        self._joint_state_sub = self.create_subscription(JointState, "joint_states", self._joint_state_callback, 10)
        self._cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)

        # handlers
        self._gyro_handler = GyroHandler()
        self._gravity_handler = GravityHandler()
        self._joints_handler = JointsHandler()
        self._command_handler = CommandHandler()

    def _imu_callback(self, msg):
        self._gyro_handler.imu_callback(msg)
        self._gravity_handler.imu_callback(msg)

    def _joint_state_callback(self, msg):
        self._joints_handler.joint_state_callback(msg)

    def _cmd_vel_callback(self, msg):
        self._command_handler.cmd_vel_callback(msg)
