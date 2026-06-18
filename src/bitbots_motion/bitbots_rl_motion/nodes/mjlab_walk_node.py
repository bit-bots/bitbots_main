import numpy as np
from handlers.command_handler import CommandHandler
from handlers.gravity_handler import GravityHandler
from handlers.gyro_handler import GyroHandler
from handlers.joint_handler import JointHandler
from handlers.robot_state_handler import RobotStateHandler

from bitbots_msgs.msg import JointCommand
from nodes.rl_node import RLNode, create_main


class MjLabWalkNode(RLNode):
    def __init__(self):
        # Configuring self._phase, self._previous_action
        super().__init__(node_name="mjlab_walk_node")

        # publishers
        self._joint_command_pub = self.create_publisher(JointCommand, "walking_motor_goals", 10)

        # handlers
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._joint_handler = JointHandler(self)
        self._robot_state_handler = RobotStateHandler(self)
        self._command_handler = CommandHandler(self)

        # loading model
        model = self.get_parameter("model").value
        self.load_model(model)

    # observations
    def obs(self):
        observation = np.hstack(
            [
                self._gyro_handler.get_gyro(),
                self._gravity_handler.get_gravity(),
                self._joint_handler.get_angle_data(),
                self._joint_handler.get_velocity_data(),
                self._previous_action.get_previous_action(),
                self._phase.get_obs_phase(),
                self._command_handler.get_command(),
            ]
        ).astype(np.float32)

        return observation

    # publisher function
    def publisher(self, onnx_pred):
        joint_command = self._joint_handler.get_joint_commands(onnx_pred)
        self._joint_command_pub.publish(joint_command)

    # states in which the policy executes
    def allowed_states(self):
        allowed_to_move = self._robot_state_handler.is_walkable()
        return allowed_to_move

    def _phase_init_hook(self):
        self._phase.set_obs_phase(
            np.array([np.sin(self._phase.get_phase()), np.cos(self._phase.get_phase())], dtype=np.float32).flatten()
        )

    def _phase_update_hook(self):
        # get cmd, if less then threshold, set phase to 0 and obs to [0,0]
        if np.linalg.norm(self._command_handler.get_command()) < 0.01:
            self._phase.set_phase(np.array([0], dtype=np.float32))
            self._phase.set_obs_phase(np.array([0,0], dtype=np.float32))
        else:
            phase = self._phase.get_phase()
            new_phase = phase + self._phase.get_phase_dt()
            self._phase.set_phase(np.fmod(new_phase + np.pi, 2 * np.pi) - np.pi)
            self._phase.set_obs_phase(
                np.array([np.sin(self._phase.get_phase()), np.cos(self._phase.get_phase())], dtype=np.float32).flatten()
            )



main = create_main(MjLabWalkNode)
