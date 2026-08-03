"""Standup policy node.

Wraps the HoST standup ONNX policies (trained with ``host_ground.py`` /
``pi_plus_config_ground.py``) as a ROS node. Unlike the other RL motion nodes
this one is not driven by the robot control state but by an explicit ROS
action, so the HCM DSD decides when the policy runs.

A standup needs a front (prone) and a back (supine) variant. Following
:class:`~bitbots_rl_motion.nodes.beyondmimic_standup_node.BeyondMimicStandupNode`,
both policies are loaded at startup and the ``direction`` field of the goal
selects which one runs (empty string -> the configured default variant), so a
goal only switches a reference instead of loading an ONNX session mid-fall.

Per tick the node
* builds the 73-dimensional one-step observation in the order HoST expects
  (``base_ang_vel``, ``projected_gravity``, raw ``dof_pos``, ``dof_vel``,
  previous action, ``action_rescale``),
* stacks the last ``history.length`` one-step observations into the policy
  input (438 values for the default 6 x 73), and
* publishes ``q_target = q_current + action * action_scale`` on
  ``getup_motor_goals``, which the HCM forwards to the motors while the robot
  is in the GETTING_UP state.

The first ``unactuated_steps`` ticks of every goal mirror the
``unactuated_timesteps`` warmup used during training: the observation is
zeroed, the previous action stays zero and no joint commands are published.

A goal ends when the base orientation says the robot is standing again (see
``_update_upright_state``), when the client cancels it, or — as the failure
case — when ``goal_timeout_sec`` elapses without the robot getting upright.
"""

import os
import threading

import numpy as np
import onnxruntime as rt
import rclpy
from ament_index_python import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from bitbots_msgs.action import RLStandup
from bitbots_msgs.msg import JointCommand
from bitbots_rl_motion.handlers import Handler
from bitbots_rl_motion.handlers.gravity_handler import GravityHandler
from bitbots_rl_motion.handlers.gyro_handler import GyroHandler
from bitbots_rl_motion.handlers.raw_joint_handler import RawJointHandler
from bitbots_rl_motion.nodes.rl_node import RLNode
from bitbots_rl_motion.observation_history import ObservationHistory

# How often the action's execute callback checks for cancellation/timeout.
GOAL_POLL_INTERVAL_SEC = 0.05


class StandupNode(RLNode):
    """Runtime for the HoST standup policies, triggered via a ROS action."""

    def __init__(self):
        # Configures self._phase, self._previous_action.
        super().__init__(node_name="standup_node")

        # Standup-specific parameters. The observation/action configuration is
        # shared by all variants; only the policy differs. Split these per
        # variant if a variant was trained differently.
        self.declare_parameter("obs_scales.ang_vel", 0.25)
        self.declare_parameter("obs_scales.dof_pos", 1.0)
        self.declare_parameter("obs_scales.dof_vel", 0.05)
        self.declare_parameter("action_scale", 0.25)
        self.declare_parameter("action_rescale_obs", 1.0)
        self.declare_parameter("history.length", 6)
        self.declare_parameter("history.one_step_obs_dim", 73)
        self.declare_parameter("unactuated_steps", 30)
        self.declare_parameter("goal_timeout_sec", 8.0)
        self.declare_parameter("upright.max_tilt_deg", 30.0)
        self.declare_parameter("upright.debounce_steps", 25)

        self._ang_vel_scale = float(self.get_parameter("obs_scales.ang_vel").value)
        self._dof_pos_scale = float(self.get_parameter("obs_scales.dof_pos").value)
        self._dof_vel_scale = float(self.get_parameter("obs_scales.dof_vel").value)
        self._action_scale = float(self.get_parameter("action_scale").value)
        self._action_rescale_obs = float(self.get_parameter("action_rescale_obs").value)
        self._history_length = int(self.get_parameter("history.length").value)
        self._one_step_obs_dim = int(self.get_parameter("history.one_step_obs_dim").value)
        self._unactuated_steps = int(self.get_parameter("unactuated_steps").value)
        self._goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self._upright_max_tilt = float(np.radians(self.get_parameter("upright.max_tilt_deg").value))
        self._upright_debounce_steps = int(self.get_parameter("upright.debounce_steps").value)

        self._num_joints = len(self.get_parameter("joints.ordered_relevant_joint_names").value)

        # 3 (ang_vel) + 3 (gravity) + N (dof_pos) + N (dof_vel) + N (prev_action) + 1 (action_rescale)
        expected_one_step = 3 + 3 + self._num_joints * 3 + 1
        if expected_one_step != self._one_step_obs_dim:
            self.get_logger().warning(
                f"history.one_step_obs_dim ({self._one_step_obs_dim}) does not match the dimension "
                f"expected for {self._num_joints} joints ({expected_one_step}). "
                "Check the config against the ONNX policy interface."
            )

        # --- variant configuration (front/back) ---
        self.declare_parameter("standup.variants", ["front", "back"])
        self.declare_parameter("standup.default_variant", "back")
        self._variants = list(self.get_parameter("standup.variants").value)
        self._default_variant = str(self.get_parameter("standup.default_variant").value)

        # The HCM forwards this topic to the motors while the robot is getting up.
        self._joint_command_pub = self.create_publisher(JointCommand, "getup_motor_goals", 10)

        # Handlers. No command/ball handler: the standup policy has no command
        # input. No robot state handler either: the action goal is the trigger,
        # and the HCM already drops getup goals outside of GETTING_UP.
        self._gyro_handler = GyroHandler(self)
        self._gravity_handler = GravityHandler(self)
        self._raw_joint_handler = RawJointHandler(self, action_scale=self._action_scale)

        self._history = ObservationHistory(length=self._history_length, one_step_dim=self._one_step_obs_dim)

        # Per-variant policy, loaded once at startup so a goal only has to swap
        # references instead of loading an ONNX session while the robot lies on
        # the ground.
        self._sessions: dict[str, tuple] = {}
        for variant in self._variants:
            self.declare_parameter(f"standup.{variant}.model", "")
            self._sessions[variant] = self._load_session(variant, self.get_parameter(f"standup.{variant}.model").value)

        if self._default_variant not in self._sessions:
            raise ValueError(
                f"standup.default_variant '{self._default_variant}' is not one of standup.variants {self._variants}"
            )
        self._select_variant(self._default_variant)

        # Goal state. Written from the action server thread, read by the timer.
        self._goal_lock = threading.Lock()
        self._active = False
        self._tick = 0
        # Set by the policy timer once the robot has been upright long enough,
        # waited on by the execute callback. An Event rather than a bool so the
        # goal ends on the same tick instead of on the next poll interval.
        self._standing = threading.Event()
        self._upright_streak = 0

        self._action_server = ActionServer(
            self,
            RLStandup,
            "rl_standup",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            # Own callback group so the blocking execute callback below does not
            # starve the policy timer.
            callback_group=ReentrantCallbackGroup(),
        )

        # RLNode.load_model() is not used here: it loads a single model, while
        # this node manages one session per variant. The rest of what it does
        # (collect the handlers for the sensor check, start the policy timer) is
        # still needed, so it happens here instead. Has to come last, after all
        # handlers exist.
        self._handlers = [handler for handler in self.__dict__.values() if isinstance(handler, Handler)]
        self._timer = self.create_timer(self.get_parameter("phase.control_dt").value, self._timer_callback)

    # ------------------------------------------------------------------ variants

    def _load_session(self, variant: str, model: str) -> tuple:
        path = os.path.join(get_package_share_directory("bitbots_rl_motion"), "models", model)
        self.get_logger().info(f"Loading ONNX model for standup variant '{variant}' from {path}")
        session = rt.InferenceSession(path, providers=self.get_parameter("providers").value)

        # Catch a mismatched export early instead of on the first goal, when the
        # robot is already lying on the ground waiting to get up.
        expected_obs = self._history_length * self._one_step_obs_dim
        obs_dim = session.get_inputs()[0].shape[-1]
        action_dim = session.get_outputs()[0].shape[-1]
        if isinstance(obs_dim, int) and obs_dim != expected_obs:
            self.get_logger().error(
                f"Standup variant '{variant}' ({model}) expects an observation of {obs_dim} values, "
                f"but the config builds {expected_obs} "
                f"({self._history_length} x {self._one_step_obs_dim}). This policy will not work."
            )
        if isinstance(action_dim, int) and action_dim != self._num_joints:
            self.get_logger().error(
                f"Standup variant '{variant}' ({model}) produces {action_dim} actions, "
                f"but {self._num_joints} joints are configured. This policy will not work."
            )

        return session, [inp.name for inp in session.get_inputs()], [out.name for out in session.get_outputs()]

    def _select_variant(self, variant: str) -> None:
        self._onnx_session, self._onnx_input_name, self._onnx_output_name = self._sessions[variant]

    def _resolve_variant(self, direction: str):
        """Map a goal's ``direction`` field to a known variant key (empty -> default).

        Returns ``None`` for an unknown variant so the goal can be rejected.
        """
        key = direction.strip() if direction else ""
        if not key:
            key = self._default_variant
        if key not in self._sessions:
            return None
        return key

    # ------------------------------------------------------------------- policy

    def obs(self) -> np.ndarray:
        if self._tick < self._unactuated_steps:
            # HoST trains the unactuated warmup with a fully zeroed observation.
            one_step = np.zeros(self._one_step_obs_dim, dtype=np.float32)
        else:
            one_step = np.hstack(
                [
                    self._gyro_handler.get_gyro() * self._ang_vel_scale,
                    self._gravity_handler.get_gravity(),
                    self._raw_joint_handler.get_raw_angle_data() * self._dof_pos_scale,
                    self._raw_joint_handler.get_velocity_data() * self._dof_vel_scale,
                    self._previous_action.get_previous_action(),
                    np.array([self._action_rescale_obs], dtype=np.float32),
                ]
            ).astype(np.float32)

        self._history.update(one_step)
        return self._history.get()

    def publisher(self, onnx_pred: np.ndarray) -> None:
        self._update_upright_state()

        in_warmup = self._tick < self._unactuated_steps
        self._tick += 1

        if in_warmup:
            # Mirror the unactuated warmup: stay off the motors and keep the
            # previous-action slot of the observation at zero. RLNode has
            # already stored onnx_pred as the previous action, so undo that.
            self._previous_action.set_previous_action(np.zeros_like(onnx_pred))
            return

        self._joint_command_pub.publish(self._raw_joint_handler.get_joint_commands(onnx_pred))

    def _update_upright_state(self) -> None:
        """Track how long the robot has been standing upright.

        The policy has no done signal of its own, so the base orientation is
        what ends a goal. ``GravityHandler`` returns the world down vector in
        the base frame, which is ``[0, 0, -1]`` when the robot stands perfectly
        upright, so the tilt away from vertical is ``arccos(-gravity_z)``.

        The tilt has to stay below the threshold for ``upright.debounce_steps``
        consecutive control steps. Without that, a single frame passing through
        vertical while the robot is still swinging up would already end the
        goal and cut the motion short.
        """
        tilt = float(np.arccos(np.clip(-self._gravity_handler.get_gravity()[2], -1.0, 1.0)))

        if tilt > self._upright_max_tilt:
            self._upright_streak = 0
            return

        self._upright_streak += 1
        if self._upright_streak >= self._upright_debounce_steps and not self._standing.is_set():
            self.get_logger().info(
                f"Standup finished: upright within {np.degrees(tilt):.0f}° for {self._upright_streak} steps."
            )
            self._standing.set()

    def allowed_states(self) -> bool:
        # The policy runs while a goal is active, up to the tick on which the
        # robot was detected as standing. Checked before obs/inference, so no
        # further motor goals go out once that happened.
        return self._active and not self._standing.is_set()

    def initialize_observation(self) -> None:
        # Called by RLNode on every inactive -> active transition, i.e. once per
        # accepted goal, before the first inference of that goal.
        self._history.reset()
        self._previous_action.set_previous_action(np.zeros(self._num_joints, dtype=np.float32))
        self._tick = 0

    def _phase_update_hook(self) -> None:
        # Standup uses no gait phase.
        pass

    # ------------------------------------------------------------ action server

    def _goal_callback(self, goal_request: RLStandup.Goal) -> GoalResponse:
        if self._resolve_variant(goal_request.direction) is None:
            self.get_logger().warning(
                f"Rejecting standup goal: unknown direction '{goal_request.direction}' (known: {self._variants})"
            )
            return GoalResponse.REJECT

        with self._goal_lock:
            if self._active:
                self.get_logger().warning("Rejecting standup goal: another goal is already active.")
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel requested for standup goal.")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> RLStandup.Result:
        variant = self._resolve_variant(goal_handle.request.direction)
        if variant is None:
            # Can only happen if the parameters changed between goal acceptance
            # and execution.
            goal_handle.abort()
            return RLStandup.Result(successful=False)

        self.get_logger().info(f"Standup goal accepted; running the '{variant}' policy.")
        with self._goal_lock:
            # Selected and cleared before activating, so the timer never infers
            # with the session of the previous goal and never sees a stale
            # standing flag (which would gate allowed_states() off immediately).
            self._select_variant(variant)
            self._standing.clear()
            self._upright_streak = 0
            self._active = True

        # A goal ends when the robot is detected as standing, when the client
        # cancels it (e.g. the HCM no longer detects the robot as fallen), or
        # when the timeout elapses. The timeout is the failure case: the policy
        # did not get the robot upright, so the DSD gets control back and can
        # re-evaluate instead of the goal hanging here forever.
        start_time = self.get_clock().now()
        timeout = Duration(seconds=self._goal_timeout_sec)
        feedback = RLStandup.Feedback()

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Standup goal canceled.")
                    return RLStandup.Result(successful=False)

                if self._standing.is_set():
                    goal_handle.succeed()
                    return RLStandup.Result(successful=True)

                elapsed = self.get_clock().now() - start_time
                if elapsed >= timeout:
                    self.get_logger().warning(
                        f"Standup goal hit its {self._goal_timeout_sec:.1f}s timeout without the robot "
                        "standing up; returning control to the HCM."
                    )
                    goal_handle.abort()
                    return RLStandup.Result(successful=False)

                feedback.percent_done = int(min(100, 100 * elapsed.nanoseconds // timeout.nanoseconds))
                goal_handle.publish_feedback(feedback)

                # Waiting on the event rather than sleeping returns the moment
                # the robot is detected as standing. Deliberately not a ROS
                # rate: the poll interval must not depend on sim time,
                # otherwise a paused simulation would make this loop miss
                # cancellations.
                self._standing.wait(GOAL_POLL_INTERVAL_SEC)
        finally:
            with self._goal_lock:
                self._active = False

        goal_handle.abort()
        return RLStandup.Result(successful=False)


def main():
    rclpy.init()

    node = StandupNode()

    # MultiThreadedExecutor instead of RLNode.create_main's EventsExecutor: the
    # action's execute callback blocks for the duration of a goal and must not
    # block the policy timer.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
