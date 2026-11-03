import numpy as np

from bitbots_msgs.msg import JointCommand
from bitbots_rl_motion.handlers.command_handler import CommandHandler
from bitbots_rl_motion.handlers.gait_clock_handler import GaitClockHandler
from bitbots_rl_motion.handlers.gravity_handler import GravityHandler
from bitbots_rl_motion.handlers.gyro_handler import GyroHandler
from bitbots_rl_motion.handlers.joint_handler import JointHandler
from bitbots_rl_motion.handlers.robot_state_handler import RobotStateHandler
from bitbots_rl_motion.nodes.rl_node import RLNode, create_main
from bitbots_rl_motion.piplus_amp_utils import ActionLimiter, walk_observation


class PiplusAmpWalkNode(RLNode):
    """Runs the Pi Plus AMP velocity walk policy with a gait clock.

    The policy has one action per joint and one extra action (the last one)
    that sets the gait clock frequency. The gait clock runs in this node,
    as it did in the training environment. The observation is one frame
    without history. It also contains the full last action (joints and clock)
    and a stand flag that is 1 when the velocity command is zero.
    """

    def __init__(self):
        # Configuring self._phase, self._previous_action
        super().__init__(node_name="piplus_amp_walk_node")

        self.declare_parameter("gait.frequency_min", 1.0)
        self.declare_parameter("gait.frequency_max", 2.0)
        self.declare_parameter("gait.initial_phase", 0.0)
        self.declare_parameter("command.min", [-1.0, -1.0, -1.0])
        self.declare_parameter("command.max", [1.0, 1.0, 1.0])
        self.declare_parameter("command.stand_settle_time", 1.0)
        number_of_joints = len(self.get_parameter("joints.ordered_relevant_joint_names").value)
        self.declare_parameter("joints.position_limits_lower", [-1e6] * number_of_joints)
        self.declare_parameter("joints.position_limits_upper", [1e6] * number_of_joints)
        self.declare_parameter("joints.soft_limit_factor", 1.0)
        self.declare_parameter("joints.action_limit", 1e6)

        self._number_of_joints = number_of_joints
        self._command_min = np.array(self.get_parameter("command.min").value, dtype=np.float32)
        self._command_max = np.array(self.get_parameter("command.max").value, dtype=np.float32)
        self._stand_settle_time = float(self.get_parameter("command.stand_settle_time").value)
        self._last_moving_command_time = None

        self._action_limiter = ActionLimiter(
            self.get_parameter("joints.position_limits_lower").value,
            self.get_parameter("joints.position_limits_upper").value,
            float(self.get_parameter("joints.soft_limit_factor").value),
            self.get_parameter("joints.walkready_state").value,
            self.get_parameter("joints.action_scales").value,
            float(self.get_parameter("joints.action_limit").value),
        )

        # publishers
        self._joint_command_pub = self.create_publisher(JointCommand, "walking_motor_goals", 10)

        # handlers
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._joint_handler = JointHandler(self)
        self._robot_state_handler = RobotStateHandler(self)
        self._command_handler = CommandHandler(self)
        self._gait_clock = GaitClockHandler(
            float(self.get_parameter("gait.frequency_min").value),
            float(self.get_parameter("gait.frequency_max").value),
            float(self.get_parameter("phase.control_dt").value),
            float(self.get_parameter("gait.initial_phase").value),
        )
        self._last_action = np.zeros(self._number_of_joints + 1, dtype=np.float32)

        # loading model
        model = self.get_parameter("model").value
        self.load_model(model)

    def _get_command(self) -> np.ndarray:
        # Keep the command inside the range of the training
        return np.clip(self._command_handler.get_command(), self._command_min, self._command_max)

    # observations
    def obs(self):
        return walk_observation(
            self._gyro_handler.get_gyro(),
            self._gravity_handler.get_gravity(),
            self._joint_handler.get_angle_data(),
            self._joint_handler.get_velocity_data(),
            self._previous_action.get_previous_action(),
            self._get_command(),
            self._gait_clock.get_obs(),
        )

    # publisher function
    def publisher(self, onnx_pred):
        self._last_action = onnx_pred

        feedback, feedback_bounded = self._action_limiter.bound_feedback(onnx_pred)
        if feedback_bounded:
            self.get_logger().warning(
                f"Action of {np.abs(onnx_pred).max():.1f} is outside the feedback limit. "
                "The policy sees an observation that it did not see in training.",
                throttle_duration_sec=2.0,
            )
            self._previous_action.set_previous_action(feedback)

        joint_action, joint_action_clipped = self._action_limiter.clip_joint_action(onnx_pred[: self._number_of_joints])
        if joint_action_clipped:
            self.get_logger().warning(
                "Joint targets clipped to the soft joint limits.",
                throttle_duration_sec=2.0,
            )
        joint_command = self._joint_handler.get_joint_commands(joint_action)
        self._joint_command_pub.publish(joint_command)

    # states in which the policy executes
    def allowed_states(self):
        if not self._robot_state_handler.is_walkable():
            return False
        now = self.get_clock().now()
        if np.any(self._command_handler.get_command() != 0.0):
            self._last_moving_command_time = now
            return True
        # The policy was trained to stand still at a zero command. Let it run for
        # a short time after the command becomes zero, so that the robot can
        # finish its step and settle in the stand pose.
        if self._last_moving_command_time is None:
            return False
        return (now - self._last_moving_command_time).nanoseconds / 1e9 < self._stand_settle_time

    def initialize_observation(self):
        # The training resets the last action (joints and clock) to zero
        self._previous_action.set_previous_action(np.zeros(self._number_of_joints + 1, dtype=np.float32))
        self._last_action = np.zeros(self._number_of_joints + 1, dtype=np.float32)
        self._gait_clock.reset()

    def _phase_update_hook(self):
        # Apply the frequency action of this step after the observation of this step
        self._gait_clock.advance(float(self._last_action[self._number_of_joints]))


main = create_main(PiplusAmpWalkNode)
