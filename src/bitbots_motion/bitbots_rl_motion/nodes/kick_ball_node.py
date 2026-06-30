import numpy as np
import rclpy
from bitbots_rl_motion.history_buffer import HistoryBuffer
from handlers.gravity_handler import GravityHandler
from handlers.gyro_handler import GyroHandler
from handlers.joint_handler import JointHandler
from handlers.robot_state_handler import RobotStateHandler
from handlers.soccer_command_handler import SoccerCommandHandler
from rclpy.executors import MultiThreadedExecutor

from bitbots_msgs.msg import JointCommand
from nodes.rl_node import RLNode


class KickBallNode(RLNode):
    def __init__(self):
        # Configuring self._phase, self._previous_action
        super().__init__(node_name="kick_ball_node")

        self.declare_parameter("obs.ang_vel_scale", 0.25)
        self.declare_parameter("obs.joint_vel_scale", 0.05)
        self.declare_parameter("obs.history_length", 8)

        self.declare_parameter("command.kick_timeout", 2.0)
        self.declare_parameter("command.post_kick_stand_duration", 0.5)
        self.declare_parameter("command.pub_period", 5)
        self.declare_parameter("command.history_samples", 10)

        self._ang_vel_scale = self.get_parameter("obs.ang_vel_scale").value
        self._joint_vel_scale = self.get_parameter("obs.joint_vel_scale").value
        history_length = self.get_parameter("obs.history_length").value

        # joint mutex handled by HCM
        self._joint_command_pub = self.create_publisher(JointCommand, "kick_motor_goals", 10)

        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._joint_handler = JointHandler(self)
        self._robot_state_handler = RobotStateHandler(self)
        self._soccer_command_handler = SoccerCommandHandler(self)

        self._ang_vel_hist = HistoryBuffer(history_length)
        self._gravity_hist = HistoryBuffer(history_length)
        self._joint_pos_hist = HistoryBuffer(history_length)
        self._joint_vel_hist = HistoryBuffer(history_length)
        self._action_hist = HistoryBuffer(history_length)

        # loading model
        model = self.get_parameter("model").value
        self.load_model(model)

    def obs(self):
        # scaled single-frame terms pushed into their 8-frame histories
        self._ang_vel_hist.append(self._gyro_handler.get_gyro() * self._ang_vel_scale)
        self._gravity_hist.append(self._gravity_handler.get_gravity())
        self._joint_pos_hist.append(self._joint_handler.get_angle_data())
        self._joint_vel_hist.append(self._joint_handler.get_velocity_data() * self._joint_vel_scale)
        # previous action (a_{t-1}); RLNode keeps this up to date after each step
        self._action_hist.append(self._previous_action.get_previous_action())

        # advance the soccer command / ball histories once for this step
        self._soccer_command_handler.update()

        observation = np.hstack(
            [
                self._ang_vel_hist.buffer.flatten(),  # 3 * 8  = 24
                self._gravity_hist.buffer.flatten(),  # 3 * 8  = 24
                self._soccer_command_handler.get_soccer_command_obs(),  # 5 * 10 = 50
                self._joint_pos_hist.buffer.flatten(),  # 22 * 8 = 176
                self._joint_vel_hist.buffer.flatten(),  # 22 * 8 = 176
                self._action_hist.buffer.flatten(),  # 22 * 8 = 176
                self._soccer_command_handler.get_ball_obs(),  # 2 * 10 = 20
            ]
        ).astype(np.float32)
        return observation

    # publisher function
    def publisher(self, onnx_pred):
        joint_command = self._joint_handler.get_joint_commands(onnx_pred)
        self._joint_command_pub.publish(joint_command)

    # states in which the policy executes. It only runs while an rl_kick action
    # is holding the kick active (like the legacy kick_node gates on its kick
    # handler), and only in a kickable robot state. Outside an active kick the
    # policy is dormant and publishes nothing, so kick_motor_goals go stale and
    # the HCM drops out of KICKING back to CONTROLLABLE.
    def allowed_states(self):
        return self._robot_state_handler.is_kickable() and self._soccer_command_handler.is_kick_active()

    def initialize_observation(self):
        # Clear all 8-frame observation histories and the soccer command/ball
        # histories so the first observation after activation re-saturates them
        # with fresh sensor data instead of stale pre-activation frames.
        self._ang_vel_hist.reset()
        self._gravity_hist.reset()
        self._joint_pos_hist.reset()
        self._joint_vel_hist.reset()
        self._action_hist.reset()
        self._soccer_command_handler.reset()
        # Start the previous-action feedback term from zero on (re)activation.
        self._previous_action.set_previous_action(np.zeros_like(self._previous_action.get_previous_action()))

    def _phase_update_hook(self):
        # This policy does not use a gait phase.
        pass


def main():
    rclpy.init()

    node = KickBallNode()

    # MultiThreadedExecutor so the kick action's timed execute callback (its own
    # reentrant callback group) runs concurrently with the 50 Hz control timer.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
