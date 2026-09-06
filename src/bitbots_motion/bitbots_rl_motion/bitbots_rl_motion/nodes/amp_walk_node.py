import numpy as np

from bitbots_msgs.msg import JointCommand
from bitbots_rl_motion.handlers.command_handler import CommandHandler
from bitbots_rl_motion.handlers.gravity_handler import GravityHandler
from bitbots_rl_motion.handlers.gyro_handler import GyroHandler
from bitbots_rl_motion.handlers.joint_handler import JointHandler
from bitbots_rl_motion.handlers.robot_state_handler import RobotStateHandler
from bitbots_rl_motion.history_buffer import HistoryBuffer
from bitbots_rl_motion.nodes.rl_node import RLNode, create_main


class AmpWalkNode(RLNode):
    """Runs the AMP locomotion policy.

    The policy observes a stack of the last frames, where every frame holds the
    whole observation. The order of the terms inside a frame and the order of the
    frames follow the observation group of the training task, so the flattened
    observation is the frames from oldest to newest, each holding the terms in the
    order they are stacked below.
    """

    def __init__(self):
        # Configuring self._phase, self._previous_action
        super().__init__(node_name="amp_walk_node")

        self.declare_parameter("obs.history_length", 4)

        # publishers
        self._joint_command_pub = self.create_publisher(JointCommand, "walking_motor_goals", 10)

        # handlers
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._joint_handler = JointHandler(self)
        self._robot_state_handler = RobotStateHandler(self)
        self._command_handler = CommandHandler(self)

        self._observation_history = HistoryBuffer(self.get_parameter("obs.history_length").value)

        # loading model
        model = self.get_parameter("model").value
        self.load_model(model)

    def obs(self):
        frame = np.hstack(
            [
                self._gyro_handler.get_gyro(),
                self._gravity_handler.get_gravity(),
                self._command_handler.get_command(),
                self._joint_handler.get_angle_data(),
                self._joint_handler.get_velocity_data(),
                self._previous_action.get_previous_action(),
            ]
        ).astype(np.float32)
        self._observation_history.append(frame)
        # Oldest frame first, which is how the frames were stacked during training
        return self._observation_history.buffer.flatten()

    # publisher function
    def publisher(self, onnx_pred):
        joint_command = self._joint_handler.get_joint_commands(onnx_pred)
        self._joint_command_pub.publish(joint_command)

    # states in which the policy executes
    def allowed_states(self):
        return self._robot_state_handler.is_walkable() and np.any(self._command_handler.get_command() != 0.0)

    def initialize_observation(self):
        # Rebuild the history from fresh sensor data instead of frames from before
        # the activation, and start the previous action from zero.
        self._observation_history.reset()
        self._previous_action.set_previous_action(np.zeros_like(self._previous_action.get_previous_action()))

    def _phase_update_hook(self):
        # This policy does not use a gait phase.
        pass


main = create_main(AmpWalkNode)
