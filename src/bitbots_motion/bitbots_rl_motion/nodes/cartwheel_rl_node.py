import asyncio

import numpy as np
import rclpy
from handlers.gyro_handler import GyroHandler
from handlers.joint_handler import JointHandler
from handlers.motion_handler import MotionHandler
from handlers.orientation_handler import OrientationHandler
from handlers.robot_state_handler import RobotStateHandler
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool

from bitbots_msgs.action import BeyondMimic
from bitbots_msgs.msg import JointCommand
from nodes.rl_node import RLNode


class CartwheelRLNode(RLNode):
    """Runs a BeyondMimic whole-body tracking policy (cartwheel) as a one-shot,
    action-triggered motion.

    Unlike the walk node, the control loop is not free-running: it is driven by a
    ``BeyondMimic`` action goal that plays the reference clip from start to end and then
    succeeds. The deployed ONNX is the action-only export (single ``obs`` input, no
    ``time_step``), so the base ``RLNode`` inference path is reused unchanged.
    """

    def __init__(self):
        # Configures self._phase, self._previous_action and declares the joints.* params.
        super().__init__(node_name="cartwheel_rl_node")

        self.declare_parameter("motion.file", "tessa_rad.npz")
        # Publish to the HCM-routed acrobatic goals by default; point this at
        # "joint_command" for a quick standalone test without the HCM.
        self.declare_parameter("publish_topic", "acrobatic_motor_goals")
        # Set to false once the HCM ACROBATIC decision is wired up.
        self.declare_parameter("ignore_robot_state", True)

        self._ignore_robot_state = self.get_parameter("ignore_robot_state").value

        # publisher
        self._joint_command_pub = self.create_publisher(
            JointCommand, self.get_parameter("publish_topic").value, 10
        )

        # handlers
        self._gyro_handler = GyroHandler(self)
        self._joint_handler = JointHandler(self)
        self._orientation_handler = OrientationHandler(self)
        self._motion_handler = MotionHandler(self)
        self._robot_state_handler = RobotStateHandler(self)

        # loading model (also collects the handlers and creates the base timer)
        self.load_model(self.get_parameter("model").value)
        # The cartwheel is action-driven, not free-running: stop the base timer loop.
        self._timer.cancel()

        self._num_joints = len(self.get_parameter("joints.ordered_relevant_joint_names").value)
        self._control_dt = self.get_parameter("phase.control_dt").value

        # Service client to signal the HCM to enter/exit ACROBATIC state
        self._hcm_acrobatic_mode = self.create_client(SetBool, "perform_acrobatic_mode")

        self._action_server = ActionServer(
            self,
            BeyondMimic,
            "beyondmimic",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

    # --- action callbacks ---
    def _goal_cb(self, goal_request):
        ready, missing = self._all_sensors_ready()
        if not ready:
            self.get_logger().warning(f"Rejecting BeyondMimic goal, sensor not ready: {missing}")
            return GoalResponse.REJECT
        if not self.allowed_states():
            self.get_logger().warning("Rejecting BeyondMimic goal: robot not in an allowed state")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        result = BeyondMimic.Result()

        # Signal HCM to enter acrobatic mode (only succeeds if robot is CONTROLLABLE)
        if not self._hcm_acrobatic_mode.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("perform_acrobatic_mode service not available, aborting")
            goal_handle.abort()
            result.result = BeyondMimic.Result.ABORTED
            return result

        hcm_response: SetBool.Response = await self._hcm_acrobatic_mode.call_async(
            SetBool.Request(data=True)
        )
        if not hcm_response.success:
            self.get_logger().warning(f"HCM refused acrobatic mode: {hcm_response.message}")
            goal_handle.abort()
            result.result = BeyondMimic.Result.ABORTED
            return result

        # Start the clip at frame 0 and align the motion world frame to the robot's
        # current orientation, so the start heading is irrelevant.
        self._previous_action.set_previous_action(np.zeros(self._num_joints, dtype=np.float32))
        self._motion_handler.start()
        self._orientation_handler.capture_alignment(self._motion_handler.get_ref_anchor_quat())

        while rclpy.ok() and not self._motion_handler.finished():
            if goal_handle.is_cancel_requested:
                self._motion_handler.stop()
                await self._hcm_acrobatic_mode.call_async(SetBool.Request(data=False))
                goal_handle.canceled()
                result.result = BeyondMimic.Result.ABORTED
                return result
            if not self.allowed_states():
                self.get_logger().warning("BeyondMimic aborted: robot left an allowed state.")
                self._motion_handler.stop()
                await self._hcm_acrobatic_mode.call_async(SetBool.Request(data=False))
                goal_handle.abort()
                result.result = BeyondMimic.Result.ABORTED
                return result

            observation = self.obs()
            onnx_input = {self._onnx_input_name[0]: observation.reshape(1, -1)}
            onnx_pred = self._onnx_session.run(self._onnx_output_name, onnx_input)[0][0]
            self._previous_action.set_previous_action(onnx_pred)
            self.publisher(onnx_pred)

            self._motion_handler.advance()

            feedback = BeyondMimic.Feedback()
            feedback.progress = float(self._motion_handler.progress())
            goal_handle.publish_feedback(feedback)

            await asyncio.sleep(self._control_dt)

        self._motion_handler.stop()
        await self._hcm_acrobatic_mode.call_async(SetBool.Request(data=False))
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
        return self._robot_state_handler.is_acrobatic()

    def _phase_update_hook(self):
        # Control is driven by the action loop, not by the base timer.
        pass


def main():
    rclpy.init()
    node = CartwheelRLNode()
    # MultiThreadedExecutor so the action execute callback (which blocks on asyncio.sleep while
    # it plays the clip) runs concurrently with the sensor subscription callbacks.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
