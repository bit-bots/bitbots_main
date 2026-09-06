import numpy as np
import rclpy
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


class AmpKickNode(RLNode):
    """Runs the AMP soccer kick policy.

    The policy walks towards the ball and kicks it, so unlike the walk policy it is
    not told a velocity. Instead it observes where the ball is relative to the torso,
    together with the direction it should kick the ball to and whether a strong or a
    weak kick was asked for. How the ball moves is only given to the critic during
    training, so it is not part of the observation.

    Like the walk policy it observes a stack of the last frames, flattened from the
    oldest to the newest frame.
    """

    def __init__(self):
        # Configuring self._phase, self._previous_action
        super().__init__(node_name="amp_kick_node")

        self.declare_parameter("obs.history_length", 4)

        self.declare_parameter("ball.topic", "ball_position_relative_filtered")
        self.declare_parameter("ball.anchor_frame", "torso_link")
        self.declare_parameter("ball.timeout", 0.5)
        self.declare_parameter("ball.max_covariance", 2.0)

        self.declare_parameter("command.action_name", "rl_kick")
        self.declare_parameter("command.kick_timeout", 4.0)
        self.declare_parameter("command.strong_kick_min_strength", 2.5)
        self.declare_parameter("command.odom_frame", "odom")
        self.declare_parameter("command.base_frame", "base_link")

        # The kick is published through its own topic, the joint mutex is handled by the HCM
        self._joint_command_pub = self.create_publisher(JointCommand, "kick_motor_goals", 10)

        # handlers
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._joint_handler = JointHandler(self)
        self._robot_state_handler = RobotStateHandler(self)
        self._ball_handler = BallHandler(self)
        self._kick_handler = AmpKickHandler(self)

        self._observation_history = HistoryBuffer(self.get_parameter("obs.history_length").value)

        # loading model
        model = self.get_parameter("model").value
        self.load_model(model)

    def is_kickable(self) -> bool:
        """Whether the robot is in a state in which it may kick. Read by the kick action."""
        return self._robot_state_handler.is_kickable()

    def obs(self):
        ball_position = self._ball_handler.get_position()
        if ball_position is None or not self._ball_handler.is_available():
            # Kicking a ball we can not see would send it in an arbitrary direction
            self._kick_handler.request_abort()
            ball_position = np.zeros(3, dtype=np.float32)

        frame = np.hstack(
            [
                self._gyro_handler.get_gyro(),
                self._gravity_handler.get_gravity(),
                self._joint_handler.get_angle_data(),
                self._joint_handler.get_velocity_data(),
                self._previous_action.get_previous_action(),
                ball_position,
                self._kick_handler.get_kick_direction(),
                self._kick_handler.get_kick_strong(),
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
        return self._robot_state_handler.is_kickable() and self._kick_handler.is_active()

    def initialize_observation(self):
        # Rebuild the history from fresh sensor data instead of frames from before
        # the activation, and start the previous action from zero.
        self._observation_history.reset()
        self._previous_action.set_previous_action(np.zeros_like(self._previous_action.get_previous_action()))

    def _phase_update_hook(self):
        # This policy does not use a gait phase.
        pass


def main():
    rclpy.init()

    node = AmpKickNode()

    # MultiThreadedExecutor so the kick action's execute callback, which holds the
    # kick in its own reentrant callback group, runs next to the control timer.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
