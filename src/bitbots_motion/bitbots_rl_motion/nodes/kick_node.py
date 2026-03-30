import numpy as np
from nodes.rl_node import RLNode
from handlers.joint_handler import JointHandler
from handlers.ball_handler import BallHandler
from handlers.gravity_handler import GravityHandler
from handlers.gyro_handler import GyroHandler
from sensor_msgs.msg import Imu, JointState
from soccer_vision_3d_msgs.msg import BallArray

from bitbots_msgs.msg import JointCommand


class KickNode(RLNode):
    def __init__(self, config_path: str):
        super().__init__(config_path, node_name="kick_node")

        # publishers
        self._joint_command_pub = self.create_publisher(JointCommand, "kick_motor_goals", 10)

        # subscribers
        self._imu_sub = self.create_subscription(Imu, "imu/data", self._imu_callback, 10)
        self._joint_state_sub = self.create_subscription(JointState, "joint_states", self._joint_state_callback, 10)
        self._ball_pos_sub = self.create_subscription(BallArray, "ball_pos", self._ball_pos_callback, 10)

        # handlers
        self._gyro_handler = GyroHandler(self._config)
        self._gravity_handler = GravityHandler(self._config)
        self._joint_handler = JointHandler(self._config)
        self._ball_handler = BallHandler(self._config)

        # loading model
        model = self._config["models"]["kick_model"]
        self.load_model(model)

    # callback functions
    def _imu_callback(self, msg):
        self._gyro_handler.imu_callback(msg)
        self._gravity_handler.imu_callback(msg)

    def _joint_state_callback(self, msg):
        self._joint_handler.joint_state_callback(msg)

    def _ball_pos_callback(self, msg):
        self._ball_handler.ball_pos_callback(msg)

    # observations
    def obs(self):
        return np.hstack(
            [
                self._gyro_handler.get_gyro(),
                self._gravity_handler.get_gravity(),
                self._joint_handler.get_velocity_data(),
                self._joint_handler.get_angle_data(),
                self._joint_handler.get_previous_action(),
                self._phase.get_phase(),
                self._ball_handler.get_ball_pos(),
            ]
        ).astype(np.float32)

    # load phase function
    def load_phase(self):
        pass

    # publisher function
    def publisher(self, onnx_pred):
        joint_command = self._joint_handler.get_joint_commands(onnx_pred)
        self._joint_command_pub.publish(joint_command)
