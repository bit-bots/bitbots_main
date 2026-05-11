"""Standup-back policy node.

Wraps the HoST standup-back ONNX policy (trained with
``host_ground.py`` / ``pi_plus_config_ground.py``) as a ROS node. The node:

* Builds the 73-dimensional one-step observation in the order HoST expects
  (``base_ang_vel``, ``projected_gravity``, raw ``dof_pos``, ``dof_vel``,
  previous action, ``action_rescale``).
* Stacks the last ``history.length`` one-step observations into the policy
  input (``ObservationHistory``).
* Mirrors the ``unactuated_timesteps`` warmup from training: for the first
  ``unactuated_steps`` ticks after a goal is accepted, the observation is
  zeroed, the previous action is forced to zero, and no joint commands are
  published.
* Exposes a ROS action server on ``/rl_standup_back`` using the existing
  ``bitbots_msgs/Dynup`` action type with ``direction = "rl_standup_back"``,
  matching the dynup action-server pattern. While a goal is active, the
  policy publishes joint targets on ``standup_back_motor_goals`` with
  ``q_target = q_current + onnx_action * action_scale``.
"""

import threading

import numpy as np
import rclpy
from handlers.gravity_handler import GravityHandler
from handlers.gyro_handler import GyroHandler
from handlers.raw_joint_handler import RawJointHandler
from handlers.robot_state_handler import RobotStateHandler
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from bitbots_msgs.action import Dynup
from bitbots_msgs.msg import JointCommand
from bitbots_rl_motion.observation_history import ObservationHistory
from nodes.rl_node import RLNode

GOAL_DIRECTION = "rl_standup_back"


class StandupBackNode(RLNode):
    def __init__(self):
        # Configures self._phase, self._previous_action; loads parameters.
        super().__init__(node_name="standup_back_node")

        # Standup-specific parameters.
        self.declare_parameter("obs_scales.ang_vel", 0.25)
        self.declare_parameter("obs_scales.dof_pos", 1.0)
        self.declare_parameter("obs_scales.dof_vel", 0.05)
        self.declare_parameter("action_scale", 1.0)
        self.declare_parameter("action_rescale_obs", 1.0)
        self.declare_parameter("history.length", 6)
        self.declare_parameter("history.one_step_obs_dim", 73)
        self.declare_parameter("unactuated_steps", 30)
        self.declare_parameter("goal_timeout_sec", 8.0)

        self._ang_vel_scale = float(self.get_parameter("obs_scales.ang_vel").value)
        self._dof_pos_scale = float(self.get_parameter("obs_scales.dof_pos").value)
        self._dof_vel_scale = float(self.get_parameter("obs_scales.dof_vel").value)
        self._action_scale = float(self.get_parameter("action_scale").value)
        self._action_rescale_obs = float(self.get_parameter("action_rescale_obs").value)
        self._history_length = int(self.get_parameter("history.length").value)
        self._one_step_obs_dim = int(self.get_parameter("history.one_step_obs_dim").value)
        self._unactuated_steps = int(self.get_parameter("unactuated_steps").value)
        self._goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)

        self._num_joints = len(self.get_parameter("joints.ordered_relevant_joint_names").value)

        # Sanity: 3 (ang_vel) + 3 (gravity) + N (dof_pos) + N (dof_vel) + N (prev_act) + 1 (action_rescale)
        expected_one_step = 3 + 3 + self._num_joints * 3 + 1
        if expected_one_step != self._one_step_obs_dim:
            self.get_logger().warning(
                f"history.one_step_obs_dim ({self._one_step_obs_dim}) does not match the "
                f"expected dimension based on joints count ({expected_one_step}). "
                "Check the config and the ONNX policy interface."
            )

        # Publisher
        self._joint_command_pub = self.create_publisher(JointCommand, "dynup_motor_goals", 10)

        # Handlers
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._raw_joint_handler = RawJointHandler(self, action_scale=self._action_scale)
        self._robot_state_handler = RobotStateHandler(self)

        # History buffer (stacked observations).
        self._history = ObservationHistory(
            length=self._history_length,
            one_step_dim=self._one_step_obs_dim,
        )

        # Goal/episode state.
        self._goal_lock = threading.Lock()
        self._active = False
        self._tick = 0
        self._was_getting_up = False

        # Action server (Dynup action type, same as the dynup pattern).
        self._action_server_callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            Dynup,
            "rl_standup_back",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_server_callback_group,
        )

        # Load model. RLNode collects all Handler instances from self.__dict__,
        # subscribes to them, and starts the timer.
        model = self.get_parameter("model").value
        self.load_model(model)

    # ------------------------------------------------------------------ obs

    def _build_one_step_obs(self) -> np.ndarray:
        gyro = self._gyro_handler.get_gyro() * self._ang_vel_scale
        gravity = self._gravity_handler.get_gravity()
        dof_pos = self._raw_joint_handler.get_raw_angle_data() * self._dof_pos_scale
        dof_vel = self._raw_joint_handler.get_velocity_data() * self._dof_vel_scale
        prev_action = self._previous_action.get_previous_action()
        action_rescale = np.array([self._action_rescale_obs], dtype=np.float32)

        return np.concatenate(
            [gyro, gravity, dof_pos, dof_vel, prev_action, action_rescale]
        ).astype(np.float32)

    def obs(self) -> np.ndarray:
        one_step = self._build_one_step_obs()

        # During the unactuated warmup, HoST trains with a fully zeroed
        # observation. Reproduce that here so the policy sees the same input
        # distribution.
        if self._active and self._tick < self._unactuated_steps:
            one_step = np.zeros_like(one_step)

        self._history.update(one_step)
        return self._history.get()

    # -------------------------------------------------------------- publish

    def publisher(self, onnx_pred: np.ndarray) -> None:
        joint_command = self._raw_joint_handler.get_joint_commands(onnx_pred)
        self._joint_command_pub.publish(joint_command)

    def allowed_states(self) -> bool:
        # Publishing is gated entirely by the active goal and warmup state.
        
        return self._robot_state_handler.is_getting_up() and self._active and self._tick >= self._unactuated_steps

    # ----------------------------------------------- override timer callback

    def _timer_callback(self):
        """Tick override: keep ``_previous_action`` zero during the warmup.

        RLNode's default callback unconditionally stores the ONNX output as
        the previous action. HoST training instead force-zeroes the action
        during the unactuated phase, so the previous-action slot in the
        observation only becomes non-zero after the warmup ends. We mirror
        that here.
        """
        sensors_ready, missing_handler = self._all_sensors_ready()
        if not sensors_ready:
            self.get_logger().warning(
                f"Waiting for all sensors to be available. Following handler hasn't got the needed information: {missing_handler}",
                throttle_duration_sec=1.0,
            )
            return

        # Reset episode state on rising edge into GETTING_UP so the warmup,
        # history, and previous_action match the per-episode setup HoST trained
        # with.
        is_getting_up = self._robot_state_handler.is_getting_up()
        if is_getting_up and not self._was_getting_up:
            self._history.reset()
            self._previous_action.set_previous_action(np.zeros(self._num_joints, dtype=np.float32))
            self._tick = 0
        self._was_getting_up = is_getting_up

        observation = self.obs()

        onnx_input = {self._onnx_input_name[0]: observation.reshape(1, -1)}
        onnx_pred = self._onnx_session.run(self._onnx_output_name, onnx_input)[0][0]

        if self.allowed_states():
            self._previous_action.set_previous_action(onnx_pred)
            self.publisher(onnx_pred)
        else:
            self._previous_action.set_previous_action(np.zeros_like(onnx_pred))

        if is_getting_up:
            self._tick += 1

    # ----------------------------------------------------- action server cbs

    def _goal_callback(self, goal_request: Dynup.Goal) -> GoalResponse:
        if goal_request.direction != GOAL_DIRECTION:
            self.get_logger().warning(
                f"Rejecting goal with direction='{goal_request.direction}'. "
                f"This server only handles direction='{GOAL_DIRECTION}'."
            )
            return GoalResponse.REJECT

        with self._goal_lock:
            if self._active:
                self.get_logger().warning(
                    "Rejecting standup-back goal: another goal is already active."
                )
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel requested for standup-back goal.")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        self.get_logger().info("Standup-back goal accepted; resetting episode state.")
        with self._goal_lock:
            self._history.reset()
            self._previous_action.set_previous_action(np.zeros(self._num_joints, dtype=np.float32))
            self._tick = 0
            self._active = True

        # Hold the action open until either:
        #   - the client cancels (e.g. HCM detects the robot is upright again,
        #     mirroring how dynup gets cancelled when no longer needed), or
        #   - the hard timeout elapses. The policy has no built-in done signal,
        #     so we cap each attempt so the DSD can re-evaluate (retry or move
        #     on) instead of hanging on this action forever.
        start_time = self.get_clock().now()
        timeout = Duration(seconds=self._goal_timeout_sec)
        rate = self.create_rate(20)
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Standup-back goal canceled.")
                    return Dynup.Result(successful=False)
                if self.get_clock().now() - start_time >= timeout:
                    self.get_logger().info(
                        f"Standup-back goal hit {self._goal_timeout_sec:.1f}s timeout; "
                        "returning control to HCM."
                    )
                    goal_handle.succeed()
                    return Dynup.Result(successful=True)
                rate.sleep()
        finally:
            with self._goal_lock:
                self._active = False
                self._tick = 0
                self._previous_action.set_previous_action(np.zeros(self._num_joints, dtype=np.float32))
                self._history.reset()
            self.destroy_rate(rate)

        goal_handle.abort()
        return Dynup.Result(successful=False)


def main():
    rclpy.init()
    node = StandupBackNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
