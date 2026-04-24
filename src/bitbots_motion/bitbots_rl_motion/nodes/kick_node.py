import numpy as np
from handlers.ball_handler import BallHandler
from handlers.gravity_handler import GravityHandler
from handlers.gyro_handler import GyroHandler
from handlers.joint_handler import JointHandler

from bitbots_msgs.msg import JointCommand
from nodes.rl_node import RLNode, create_main


class KickNode(RLNode):
    def __init__(self):
        # Configuring self._phase, self._previous_action
        super().__init__(node_name="kick_node")

        # publishers
        self._joint_command_pub = self.create_publisher(JointCommand, "kick_motor_goals", 10)

        # handlers
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._joint_handler = JointHandler(self)
        self._ball_handler = BallHandler(self)

        # loading model
        model = self.get_parameter("model").value
        self.load_model(model)

    # observations
    def obs(self):
        observation = np.hstack(
            [
                self._gyro_handler.get_gyro(),
                self._gravity_handler.get_gravity(),
                self._joint_handler.get_velocity_data(),
                self._joint_handler.get_angle_data(),
                self._previous_action.get_previous_action(),
                self._phase.get_obs_phase(),
                self._ball_handler.get_ball_pos(),
            ]
        ).astype(np.float32)

        return observation

    # load phase function
    def load_phase(self):
        pass

    # publisher function
    def publisher(self, onnx_pred):
        joint_command = self._joint_handler.get_joint_commands(onnx_pred)
        self._joint_command_pub.publish(joint_command)


main = create_main(KickNode)
