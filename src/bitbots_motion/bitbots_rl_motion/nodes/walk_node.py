import time

import numpy as np
from geometry_msgs.msg import Twist
from handlers.command_handler import CommandHandler
from handlers.gravity_handler import GravityHandler
from handlers.gyro_handler import GyroHandler
from handlers.joint_handler import JointHandler
from sensor_msgs.msg import Imu, JointState

from bitbots_msgs.msg import JointCommand
from nodes.rl_node import RLNode


class WalkNode(RLNode):
    def __init__(self, config_path: str):
        # Configuring self._config, self._phase, self._previous_action
        super().__init__(config_path, node_name="walk_node")

        # publishers
        self._joint_command_pub = self.create_publisher(JointCommand, "walking_motor_goals", 10)

        # subscribers
        self._imu_sub = self.create_subscription(Imu, "imu/data", self._imu_callback, 10)
        self._joint_state_sub = self.create_subscription(JointState, "joint_states", self._joint_state_callback, 10)
        self._cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)

        # handlers
        self._gyro_handler = GyroHandler(self._config)
        self._gravity_handler = GravityHandler(self._config)
        self._joint_handler = JointHandler(self._config)
        self._command_handler = CommandHandler(self._config)

        # loading model
        model = self._config["models"]["walk_model"]
        self.load_model(model)

    # callback functions
    def _imu_callback(self, msg):
        self._gyro_handler.imu_callback(msg)
        self._gravity_handler.imu_callback(msg)

    def _joint_state_callback(self, msg):
        self._joint_handler.joint_state_callback(msg)

    def _cmd_vel_callback(self, msg):
        self._command_handler.cmd_vel_callback(msg)

    # observations
    def obs(self):
        return np.hstack(
            [
                self._gyro_handler.get_gyro(),
                self._gravity_handler.get_gravity(),
                self._command_handler.get_command(),
                self._joint_handler.get_velocity_data(),
                self._joint_handler.get_angle_data(),
                self._previous_action.get_previous_action(),
                self._phase.get_obs_phase(),
            ]
        ).astype(np.float32)

    # load phase function
    def load_phase(self):
        timestamp = self.get_clock().now().to_msg()
        walkready_command = self._joint_handler.get_walkready_joint_command(timestamp=timestamp)
        self._joint_command_pub.publish(walkready_command)
        time.sleep(10)

    # publisher function
    def publisher(self, onnx_pred):
        joint_command = self._joint_handler.get_joint_commands(onnx_pred)
        self._joint_command_pub.publish(joint_command)
