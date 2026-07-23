import numpy as np

from bitbots_msgs.msg import JointCommand
from bitbots_rl_motion.handlers.gravity_handler import GravityHandler
from bitbots_rl_motion.handlers.gyro_handler import GyroHandler
from bitbots_rl_motion.handlers.joint_handler import JointHandler
from bitbots_rl_motion.handlers.path_handler import PathHandler
from bitbots_rl_motion.handlers.robot_state_handler import RobotStateHandler
from bitbots_rl_motion.nodes.rl_node import RLNode, create_main


class MjLabPathNode(RLNode):
    """Runtime for the mjlab Pi Plus path-following policy.

    The policy walks along a reference path given as a ``nav_msgs/Path`` instead
    of following a velocity command: its command observation is the next stretch
    of that path in the robot's base frame plus the goal orientation (see
    :class:`PathHandler`). The path is held in a configurable world frame
    (``path.frame``, odom by default) and the robot's pose inside it comes from
    tf, so the observation follows the robot along the path without the path
    itself being resent.
    """

    def __init__(self):
        # Configures self._phase, self._previous_action.
        super().__init__(node_name="mjlab_path_node")

        # Whether the gait phase observation is zeroed once the robot is on the
        # goal pose. mjlab does this via the stand flag of the (hidden) twist
        # command, but only if the policy was trained with that stand enabled --
        # a policy that never saw a zeroed phase would get an out-of-distribution
        # observation. See the config for how to tell the two apart.
        self.declare_parameter("phase.freeze_at_goal", False)
        self._freeze_at_goal = bool(self.get_parameter("phase.freeze_at_goal").value)

        # Same topic as the other walking policies; the HCM arbitrates.
        self._joint_command_pub = self.create_publisher(JointCommand, "walking_motor_goals", 10)

        # Handlers.
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._joint_handler = JointHandler(self)
        self._robot_state_handler = RobotStateHandler(self)
        self._path_handler = PathHandler(self)

        # Free-running gait clock, in radians. Kept here rather than in the
        # PhaseHandler because this policy observes the mjlab gait phase
        # ([sin, cos] of a single clock) instead of the per-leg phase pair.
        self._gait_phase = 0.0

        # Loading model.
        model = self.get_parameter("model").value
        self.load_model(model)

    def obs(self) -> np.ndarray:
        observation = np.hstack(
            [
                self._gyro_handler.get_gyro(),
                self._gravity_handler.get_gravity(),
                self._joint_handler.get_angle_data(),
                self._joint_handler.get_velocity_data(),
                self._previous_action.get_previous_action(),
                self._gait_phase_obs(),
                self._path_handler.get_command(),
            ]
        ).astype(np.float32)

        return observation

    def _gait_phase_obs(self) -> np.ndarray:
        """[sin, cos] of the gait clock, zeroed at the goal if configured.

        Mirrors the training-time phase term: the clock runs freely with the
        control loop, and the observation is zeroed while the command asks the
        robot to stand, which the path command does at the goal pose. That is
        what makes the policy plant its feet there instead of marching on the
        spot -- but only for a policy that was trained with the stand enabled.
        """
        if self._freeze_at_goal and self._path_handler.is_at_goal():
            return np.zeros(2, dtype=np.float32)
        return np.array([np.sin(self._gait_phase), np.cos(self._gait_phase)], dtype=np.float32)

    # publisher function
    def publisher(self, onnx_pred: np.ndarray) -> None:
        joint_command = self._joint_handler.get_joint_commands(onnx_pred)
        self._joint_command_pub.publish(joint_command)

    # states in which the policy executes
    def allowed_states(self) -> bool:
        return self._robot_state_handler.is_walkable() and self._path_handler.has_path()

    def _phase_update_hook(self) -> None:
        # The gait clock keeps advancing at the goal as well; only the phase
        # observation is zeroed there, exactly as in training.
        self._gait_phase = (self._gait_phase + self._phase.get_phase_dt()) % (2 * np.pi)

    def initialize_observation(self) -> None:
        # No observation history. Restart the gait clock, as an episode does in
        # training, and start the previous-action feedback term from zero.
        self._gait_phase = 0.0
        self._previous_action.set_previous_action(np.zeros_like(self._previous_action.get_previous_action()))


main = create_main(MjLabPathNode)
