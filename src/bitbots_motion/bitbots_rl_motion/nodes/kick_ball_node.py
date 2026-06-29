import numpy as np
import rclpy
from handlers.gravity_handler import GravityHandler
from handlers.gyro_handler import GyroHandler
from handlers.joint_handler import JointHandler
from handlers.robot_state_handler import RobotStateHandler
from handlers.soccer_command_handler import SoccerCommandHandler
from rclpy.executors import MultiThreadedExecutor

from bitbots_msgs.msg import JointCommand
from bitbots_rl_motion.history_buffer import HistoryBuffer
from nodes.rl_node import RLNode


class KickBallNode(RLNode):
    """Encoder/decoder soccer (walk + kick) policy for PiPlus_S_12L8A0G2H0W.

    22-DOF full-body net running at 50 Hz. The 646-dim observation stacks an
    8-frame history of base angular velocity, projected gravity, relative joint
    position, relative joint velocity and the previous action, plus a 50-dim
    soccer command and a 20-dim ball history (see SoccerCommandHandler). The net
    outputs 22 joint targets as ``action * action_scale + default_joint_pos``.
    """

    def __init__(self):
        # Configuring self._phase, self._previous_action
        super().__init__(node_name="kick_ball_node")

        # observation parameters (fixed by the trained policy)
        self.declare_parameter("obs.ang_vel_scale", 0.25)
        self.declare_parameter("obs.joint_vel_scale", 0.05)
        self.declare_parameter("obs.history_length", 8)

        # soccer command parameters
        self.declare_parameter("command.kick_timeout", 2.0)
        self.declare_parameter("command.pub_period", 5)
        self.declare_parameter("command.history_samples", 10)

        self._ang_vel_scale = self.get_parameter("obs.ang_vel_scale").value
        self._joint_vel_scale = self.get_parameter("obs.joint_vel_scale").value
        history_length = self.get_parameter("obs.history_length").value

        # publishers
        self._joint_command_pub = self.create_publisher(JointCommand, "walking_motor_goals", 10)

        # handlers
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._joint_handler = JointHandler(self)
        self._robot_state_handler = RobotStateHandler(self)
        self._soccer_command_handler = SoccerCommandHandler(self)

        # 8-frame observation histories
        self._ang_vel_hist = HistoryBuffer(history_length)
        self._gravity_hist = HistoryBuffer(history_length)
        self._joint_pos_hist = HistoryBuffer(history_length)
        self._joint_vel_hist = HistoryBuffer(history_length)
        self._action_hist = HistoryBuffer(history_length)

        # loading model
        model = self.get_parameter("model").value
        self.load_model(model)

    # observations
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

    # states in which the policy executes (it both walks and kicks)
    def allowed_states(self):
        return self._robot_state_handler.is_walkable() or self._robot_state_handler.is_kickable()

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
