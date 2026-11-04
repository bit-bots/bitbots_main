import numpy as np
import rclpy
from bitbots_tf_buffer import Buffer
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import MultiThreadedExecutor

from bitbots_msgs.msg import JointCommand
from bitbots_rl_motion.handlers.amp_kick_handler import AmpKickHandler
from bitbots_rl_motion.handlers.ball_handler import BallHandler
from bitbots_rl_motion.handlers.gravity_handler import GravityHandler
from bitbots_rl_motion.handlers.gyro_handler import GyroHandler
from bitbots_rl_motion.handlers.joint_handler import JointHandler
from bitbots_rl_motion.handlers.robot_state_handler import RobotStateHandler
from bitbots_rl_motion.history_buffer import HistoryBuffer
from bitbots_rl_motion.nodes.rl_node import RLNode
from bitbots_rl_motion.piplus_amp_utils import ActionLimiter, kick_frame


class PiplusAmpKickNode(RLNode):
    """Runs the Pi Plus AMP soccer kick policy.

    The policy walks to the ball and kicks it. It does not get a velocity
    command. It gets the ball position in the torso frame, the kick direction
    and the kick intensity. The observation is a history of frames in
    time-major order (all terms of the oldest frame first).

    The policy was trained in simulation only. Its output has no bounds, so
    this node clips the joint targets to the soft joint limits.
    """

    def __init__(self):
        # Configuring self._phase, self._previous_action
        super().__init__(node_name="piplus_amp_kick_node")

        self.declare_parameter("obs.history_length", 4)

        self.declare_parameter("ball.topic", "ball_position_relative_filtered")
        self.declare_parameter("ball.anchor_frame", "torso_link")
        self.declare_parameter("ball.timeout", 0.5)
        self.declare_parameter("ball.max_covariance", 2.0)
        self.declare_parameter("ball.position_clip", 10.0)
        # Scale of the tanh bound of the ball position. Null turns the bound off.
        # It must agree with the training of the loaded model.
        self.declare_parameter("ball.pos_tanh_scale", None, ParameterDescriptor(dynamic_typing=True))

        self.declare_parameter("command.action_name", "rl_kick")
        self.declare_parameter("command.kick_timeout", 4.0)
        self.declare_parameter("command.max_kick_intensity", 1.0)
        self.declare_parameter("command.odom_frame", "odom")
        self.declare_parameter("command.base_frame", "base_footprint")

        number_of_joints = len(self.get_parameter("joints.ordered_relevant_joint_names").value)
        self.declare_parameter("joints.position_limits_lower", [-1e6] * number_of_joints)
        self.declare_parameter("joints.position_limits_upper", [1e6] * number_of_joints)
        self.declare_parameter("joints.soft_limit_factor", 1.0)
        self.declare_parameter("joints.action_limit", 1e6)

        self._action_limiter = ActionLimiter(
            self.get_parameter("joints.position_limits_lower").value,
            self.get_parameter("joints.position_limits_upper").value,
            float(self.get_parameter("joints.soft_limit_factor").value),
            self.get_parameter("joints.walkready_state").value,
            self.get_parameter("joints.action_scales").value,
            float(self.get_parameter("joints.action_limit").value),
        )

        # joint mutex handled by HCM
        self._joint_command_pub = self.create_publisher(JointCommand, "kick_motor_goals", 10)

        self._tf_buffer = Buffer(node=self)

        # handlers
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._joint_handler = JointHandler(self)
        self._robot_state_handler = RobotStateHandler(self)
        self._ball_handler = BallHandler(self, self._tf_buffer)
        self._kick_handler = AmpKickHandler(self, self._tf_buffer, self._ball_handler.has_received_ball)

        self._observation_history = HistoryBuffer(self.get_parameter("obs.history_length").value)
        self._last_ball_position = None

        # loading model
        model = self.get_parameter("model").value
        self.load_model(model)

    def is_kickable(self) -> bool:
        """Returns True if the robot state allows a kick. The kick action reads this value."""
        return self._robot_state_handler.is_kickable()

    def _get_ball_position(self) -> np.ndarray:
        position = self._ball_handler.get_position() if self._ball_handler.is_available() else None
        if position is None:
            # A kick without a known ball sends the ball in a random direction
            self._kick_handler.request_abort()
            if self._last_ball_position is None:
                return np.zeros(3, dtype=np.float32)
            return self._last_ball_position
        self._last_ball_position = position
        return position

    def obs(self):
        frame = kick_frame(
            self._gyro_handler.get_gyro(),
            self._gravity_handler.get_gravity(),
            self._joint_handler.get_angle_data(),
            self._joint_handler.get_velocity_data(),
            self._previous_action.get_previous_action(),
            self._get_ball_position(),
            self._kick_handler.get_kick_direction(),
            self._kick_handler.get_kick_intensity(),
        )
        self._observation_history.append(frame)
        # Time-major order, oldest frame first, as in the training
        return self._observation_history.buffer.flatten()

    # publisher function
    def publisher(self, onnx_pred):
        feedback, feedback_bounded = self._action_limiter.bound_feedback(onnx_pred)
        if feedback_bounded:
            self.get_logger().warning(
                f"Action of {np.abs(onnx_pred).max():.1f} is outside the feedback limit. "
                "The policy sees an observation that it did not see in training.",
                throttle_duration_sec=2.0,
            )
            self._previous_action.set_previous_action(feedback)

        joint_action, joint_action_clipped = self._action_limiter.clip_joint_action(onnx_pred)
        if joint_action_clipped:
            self.get_logger().debug("Joint targets clipped to the soft joint limits.", throttle_duration_sec=2.0)
        joint_command = self._joint_handler.get_joint_commands(joint_action)
        self._joint_command_pub.publish(joint_command)

    # states in which the policy executes
    def allowed_states(self):
        return self._robot_state_handler.is_kickable() and self._kick_handler.is_active()

    def initialize_observation(self):
        # Fill the history again from new sensor data and start with a zero last action
        self._observation_history.reset()
        self._last_ball_position = None
        self._previous_action.set_previous_action(np.zeros_like(self._previous_action.get_previous_action()))

    def _phase_update_hook(self):
        # This policy does not use a gait phase.
        pass


def main():
    rclpy.init()

    node = PiplusAmpKickNode()

    # MultiThreadedExecutor so the kick action's timed execute callback (its own
    # reentrant callback group) runs concurrently with the 50 Hz control timer.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
