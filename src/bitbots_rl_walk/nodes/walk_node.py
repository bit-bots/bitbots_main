import time

import numpy as np
from geometry_msgs.msg import Twist
from handler.command_handler import CommandHandler
from handler.gravity_handler import GravityHandler
from handler.gyro_handler import GyroHandler
from sensor_msgs.msg import Imu, JointState

from bitbots_msgs.msg import JointCommand
from bitbots_rl_walk.confg.timer_phase_confg import TimerPhaseConfg
from bitbots_rl_walk.handler.joint_handler import JointHandler
from bitbots_rl_walk.nodes.rl_node import RLNode


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
        self._joint_handler = JointHandler()
        self._command_handler = CommandHandler()

        self._timer_phase_confg = TimerPhaseConfg()

        self.confg()

        self._obs = np.hstack(
            [
                self._gyro_handler.get_data(),  # 3
                self._gravity_handler.get_data(),  # 4
                self._command_handler.get_data(),  # 3
                self._joint_handler.get_velocity_data(),  # 18
                self._joint_handler.get_angle_data(),  # 18
                # TODO: fix
                self._joint_handler.get_previous_action(),  # 18  # Previous action
                self._timer_phase_confg.get_obs_phase(),  # 2
            ]
        ).astype(np.float32)

    def _imu_callback(self, msg):
        self._gyro_handler.imu_callback(msg)
        self._gravity_handler.imu_callback(msg)

    def _joint_state_callback(self, msg):
        self._joint_handler.joint_state_callback(msg)

    def _cmd_vel_callback(self, msg):
        self._command_handler.cmd_vel_callback(msg)

    def load_phase(self):
        walkready_command = self._joint_handler.get_walkready_joint_command()
        self._joint_command_pub.publish(walkready_command)
        time.sleep(10)

    def publisher(self, onnx_pred):
        joint_command = self._joint_handler.get_joint_commands(onnx_pred)
        self._joint_command_pub.publish(joint_command)
