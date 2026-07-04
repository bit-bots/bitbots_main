import numpy as np

from bitbots_msgs.msg import JointCommand
from bitbots_rl_motion.handlers.dribble_active_handler import DribbleActiveHandler
from bitbots_rl_motion.handlers.dribble_command_handler import DribbleCommandHandler
from bitbots_rl_motion.handlers.gravity_handler import GravityHandler
from bitbots_rl_motion.handlers.gyro_handler import GyroHandler
from bitbots_rl_motion.handlers.joint_handler import JointHandler
from bitbots_rl_motion.handlers.robot_state_handler import RobotStateHandler
from bitbots_rl_motion.nodes.rl_node import RLNode, create_main

# The policy observes the target ball speed divided by this scale; must match
# the normalization used during training (see the kick env's observation).
_SPEED_OBS_SCALE = 5.0


class DribbleNode(RLNode):
    """Runs the learned dribble/kick policy.

    The policy walks to the ball and kicks it with the inside of the foot in a
    commanded direction with a commanded strength. It is a drop-in replacement
    for the walk while active: it publishes to the same motor goal topic, uses
    the same gait conventions and shares the gait phase with the walk node via
    the phase sync topic, so the two policies can hand over mid-gait.

    API: the behavior starts/stops it with ``rl_dribble_active`` and commands
    it with ``rl_dribble_command`` (desired ball velocity vector, see
    DribbleCommandHandler). The walk node yields while the dribble is active.
    """

    def __init__(self):
        # Configuring self._phase, self._previous_action
        super().__init__(node_name="dribble_node")

        # Own goal topic: the HCM switches to the DRIBBLING state while these
        # goals arrive, which mutes the regular walking goals so the dribble
        # policy overrides the walk.
        self._joint_command_pub = self.create_publisher(JointCommand, "dribble_motor_goals", 10)

        # handlers
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._joint_handler = JointHandler(self)
        self._robot_state_handler = RobotStateHandler(self)
        self._dribble_active_handler = DribbleActiveHandler(self)
        self._dribble_command_handler = DribbleCommandHandler(self)

        # loading model
        model = self.get_parameter("model").value
        self.load_model(model)

    # observations
    def obs(self):
        # advance the ball/command transforms and history once per step
        self._dribble_command_handler.update()
        return np.hstack(
            [
                self._dribble_command_handler.get_ball_obs(),
                self._dribble_command_handler.get_ball_prev_obs(),
                self._gyro_handler.get_gyro(),
                self._gravity_handler.get_gravity(),
                self._joint_handler.get_angle_data(),
                self._previous_action.get_previous_action(),
                self._phase.get_obs_phase(),
                self._dribble_command_handler.get_kick_dir_obs(),
                np.array([self._dribble_command_handler.get_kick_speed() / _SPEED_OBS_SCALE]),
            ]
        ).astype(np.float32)

    # publisher function
    def publisher(self, onnx_pred):
        joint_command = self._joint_handler.get_joint_commands(onnx_pred)
        self._joint_command_pub.publish(joint_command)

    # states in which the policy executes
    def allowed_states(self):
        return (
            self._robot_state_handler.is_dribbleable()
            and self._dribble_active_handler.is_active()
            and self._dribble_command_handler.has_ball_and_command()
        )

    def initialize_observation(self):
        # Continue the gait from the walk's current phase for a seamless
        # transition, then restart the ball history and the previous-action
        # feedback term from fresh data.
        self._phase.adopt_external_phase()
        self._dribble_command_handler.reset()
        self._previous_action.set_previous_action(np.zeros_like(self._previous_action.get_previous_action()))

    def _phase_update_hook(self):
        if not self._phase.check_phase_set():
            return
        # The dribble gait never freezes (no stop command in training); the
        # phase advances continuously while the policy is active.
        phase_tp1 = self._phase.get_phase() + self._phase.get_phase_dt()
        self._phase.set_phase(np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi)
        self._phase.publish_phase()


main = create_main(DribbleNode)
