import os

import numpy as np
import onnxruntime as rt
import rclpy
from ament_index_python import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from bitbots_msgs.action import BeyondMimic
from bitbots_msgs.msg import JointCommand
from bitbots_rl_motion.handlers.gyro_handler import GyroHandler
from bitbots_rl_motion.handlers.joint_handler import JointHandler
from bitbots_rl_motion.handlers.motion_handler import MotionHandler
from bitbots_rl_motion.handlers.orientation_handler import OrientationHandler
from bitbots_rl_motion.handlers.robot_state_handler import RobotStateHandler
from bitbots_rl_motion.nodes.rl_node import RLNode
from bitbots_utils.utils import async_wait_for


class BeyondMimicStandupNode(RLNode):
    """Runs a BeyondMimic whole-body tracking policy as a one-shot, action-triggered
    standup (getup) motion, driven directly by the HCM.

    This mirrors :class:`~bitbots_rl_motion.nodes.cartwheel_rl_node.CartwheelRLNode`
    (same 109-dim Wo observation, same action-only ONNX export, same clip-playback
    control loop), but it is triggered from *inside* the HCM instead of from the body
    behavior. Two things follow from that:

    * The HCM owns the robot state and enters the getup state itself before sending the
      goal, so there is no ``perform_acrobatic_mode`` service handshake to switch the HCM
      into a dedicated mode. Publishing is gated on the getup state and forwarded by
      ``hcm.cpp`` on ``getup_motor_goals``.
    * A standup needs a front (prone) and a back (supine) variant. Both policies and
      reference clips are loaded at startup; the ``motion`` field of the goal selects the
      variant to play (empty string -> the configured default variant).
    """

    def __init__(self):
        # Declares the joints.* params and configures self._phase / self._previous_action.
        super().__init__(node_name="beyondmimic_standup_node")

        # --- variant configuration (front/back) ---
        self.declare_parameter("standup.variants", ["front", "back"])
        self.declare_parameter("standup.default_variant", "front")
        self._variants = list(self.get_parameter("standup.variants").value)
        self._default_variant = self.get_parameter("standup.default_variant").value

        # Abort the getup when the robot's base orientation drifts too far from the reference
        # clip: a failed getup keeps its base lying flat while the reference tracks toward
        # upright, so the anchor tracking error runs away instead of staying near 0. The
        # error must exceed the threshold for `debounce` consecutive control steps to abort,
        # which filters IMU noise and brief tracking transients during fast phases.
        # Set max_tracking_error_deg <= 0 to disable the check.
        self.declare_parameter("standup.max_tracking_error_deg", 45.0)
        self.declare_parameter("standup.tracking_error_debounce", 5)
        self._max_tracking_error = np.radians(self.get_parameter("standup.max_tracking_error_deg").value)
        self._tracking_error_debounce = int(self.get_parameter("standup.tracking_error_debounce").value)

        # hcm.cpp forwards this topic to the motors only while the robot is getting up.
        # Point this at "joint_command" for a quick standalone test without the HCM.
        self.declare_parameter("publish_topic", "getup_motor_goals")
        # Set to true only for a standalone test that bypasses the HCM robot-state gate.
        self.declare_parameter("ignore_robot_state", False)
        self._ignore_robot_state = self.get_parameter("ignore_robot_state").value

        # publisher
        self._joint_command_pub = self.create_publisher(
            JointCommand, self.get_parameter("publish_topic").value, 10
        )

        # Shared handlers (identical to the cartwheel node). The joint config
        # (order/defaults/kp/kd/scales) is assumed to be shared by both variants.
        self._gyro_handler = GyroHandler(self)
        self._joint_handler = JointHandler(self)
        self._orientation_handler = OrientationHandler(self)
        self._robot_state_handler = RobotStateHandler(self)

        # Per-variant policy + reference clip. Both are loaded once at startup so a goal
        # only has to switch references instead of re-loading an ONNX session mid-fall.
        self._sessions: dict[str, tuple] = {}
        self._motion_handlers: dict[str, MotionHandler] = {}
        for variant in self._variants:
            self.declare_parameter(f"standup.{variant}.model", "")
            self.declare_parameter(f"standup.{variant}.motion", "")
            model = self.get_parameter(f"standup.{variant}.model").value
            motion = self.get_parameter(f"standup.{variant}.motion").value
            self._sessions[variant] = self._load_session(model)
            self._motion_handlers[variant] = MotionHandler(self, motion_file=motion)

        # Collect the handlers for the _all_sensors_ready() check. The base load_model()
        # would do this, but it also loads a single model and starts a free-running timer,
        # neither of which fits the two-variant, action-driven standup.
        self._handlers = [
            self._gyro_handler,
            self._joint_handler,
            self._orientation_handler,
            self._robot_state_handler,
            *self._motion_handlers.values(),
        ]

        # Active variant references, (re)selected on every goal.
        self._motion_handler = self._motion_handlers[self._default_variant]
        (
            self._onnx_session,
            self._onnx_input_name,
            self._onnx_output_name,
        ) = self._sessions[self._default_variant]

        self._num_joints = len(self.get_parameter("joints.ordered_relevant_joint_names").value)
        self._control_dt = self.get_parameter("phase.control_dt").value

        self._action_server = ActionServer(
            self,
            BeyondMimic,
            "beyondmimic_standup",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

    def _load_session(self, model: str):
        path = os.path.join(get_package_share_directory("bitbots_rl_motion"), "models", model)
        self.get_logger().warning(f"Loading ONNX model from {path}")
        session = rt.InferenceSession(path, providers=self.get_parameter("providers").value)
        input_names = [inp.name for inp in session.get_inputs()]
        output_names = [out.name for out in session.get_outputs()]
        return session, input_names, output_names

    def _resolve_variant(self, motion: str):
        """Map the goal's ``motion`` field to a known variant key (empty -> default).

        Returns ``None`` for an unknown variant so the goal can be rejected/aborted.
        """
        key = motion.strip() if motion else ""
        if not key:
            key = self._default_variant
        if key not in self._sessions:
            return None
        return key

    # --- action callbacks ---
    def _goal_cb(self, goal_request):
        # We do NOT gate on the robot state here: the HCM is the caller and sets the getup
        # state itself, so the state may still be propagating when the goal arrives.
        ready, missing = self._all_sensors_ready()
        if not ready:
            self.get_logger().warning(f"Rejecting standup goal, sensor not ready: {missing}")
            return GoalResponse.REJECT
        if self._resolve_variant(goal_request.motion) is None:
            self.get_logger().warning(
                f"Rejecting standup goal: unknown variant '{goal_request.motion}' (known: {self._variants})"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        result = BeyondMimic.Result()

        variant = self._resolve_variant(goal_handle.request.motion)
        if variant is None:
            goal_handle.abort()
            result.result = BeyondMimic.Result.ABORTED
            return result
        self.get_logger().info(f"Performing standup motion (variant '{variant}')")

        # Select the active policy + reference clip for this variant.
        self._motion_handler = self._motion_handlers[variant]
        (
            self._onnx_session,
            self._onnx_input_name,
            self._onnx_output_name,
        ) = self._sessions[variant]

        # Start the clip at frame 0 and align the motion world frame to the robot's current
        # orientation, so the fallen heading is irrelevant.
        self._previous_action.set_previous_action(np.zeros(self._num_joints, dtype=np.float32))
        self._motion_handler.start()
        self._orientation_handler.capture_alignment(self._motion_handler.get_ref_anchor_quat())
        tracking_error_streak = 0

        while rclpy.ok() and not self._motion_handler.finished():
            if goal_handle.is_cancel_requested:
                self._motion_handler.stop()
                goal_handle.canceled()
                result.result = BeyondMimic.Result.ABORTED
                return result
            if not self.allowed_states():
                self.get_logger().warning("Standup aborted: robot left the getup state.")
                self._motion_handler.stop()
                goal_handle.abort()
                result.result = BeyondMimic.Result.ABORTED
                return result

            # Abort a diverging getup: the robot base orientation has drifted too far from
            # the reference clip for too long (e.g. the getup failed and the robot is
            # flopping on the ground instead of tracking the clip toward upright).
            if self._max_tracking_error > 0.0:
                tracking_error = self._orientation_handler.get_anchor_ori_error(
                    self._motion_handler.get_ref_anchor_quat()
                )
                if tracking_error > self._max_tracking_error:
                    tracking_error_streak += 1
                    if tracking_error_streak >= self._tracking_error_debounce:
                        self.get_logger().warning(
                            f"Standup aborted: reference tracking error "
                            f"{np.degrees(tracking_error):.0f}° exceeded "
                            f"{np.degrees(self._max_tracking_error):.0f}° for "
                            f"{tracking_error_streak} steps (frame {self._motion_handler.time_step()})."
                        )
                        self._motion_handler.stop()
                        goal_handle.abort()
                        result.result = BeyondMimic.Result.ABORTED
                        return result
                else:
                    tracking_error_streak = 0

            observation = self.obs()
            onnx_input = {self._onnx_input_name[0]: observation.reshape(1, -1)}
            onnx_pred = self._onnx_session.run(self._onnx_output_name, onnx_input)[0][0]
            self._previous_action.set_previous_action(onnx_pred)
            self.publisher(onnx_pred)

            self._motion_handler.advance()

            feedback = BeyondMimic.Feedback()
            feedback.progress = float(self._motion_handler.progress())
            goal_handle.publish_feedback(feedback)

            await async_wait_for(self, self._control_dt)

        self._motion_handler.stop()
        goal_handle.succeed()
        result.result = BeyondMimic.Result.SUCCESS
        return result

    # --- RLNode hooks ---
    def obs(self):
        return np.hstack(
            [
                self._motion_handler.get_command(),  # 40
                self._orientation_handler.get_motion_anchor_ori_b(
                    self._motion_handler.get_ref_anchor_quat()
                ),  # 6
                self._gyro_handler.get_gyro(),  # 3
                self._joint_handler.get_angle_data(),  # 20
                self._joint_handler.get_velocity_data(),  # 20
                self._previous_action.get_previous_action(),  # 20
            ]
        ).astype(np.float32)

    def publisher(self, onnx_pred):
        joint_command = self._joint_handler.get_joint_commands(onnx_pred)
        self._joint_command_pub.publish(joint_command)

    def allowed_states(self):
        if self._ignore_robot_state:
            return True
        return self._robot_state_handler.is_getup()

    def _phase_update_hook(self):
        # Control is driven by the action loop, not by the base timer.
        pass


def main():
    rclpy.init()
    node = BeyondMimicStandupNode()
    # MultiThreadedExecutor so the action execute callback (which awaits between control
    # steps while it plays the clip) runs concurrently with the sensor subscription callbacks.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
