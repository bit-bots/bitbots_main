import os
from abc import ABC, abstractmethod
from pathlib import Path

import numpy as np
import onnxruntime as rt
import rclpy
from ament_index_python import get_package_share_directory
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from bitbots_rl_motion.handlers import Handler
from bitbots_rl_motion.handlers.phase import PhaseHandler
from bitbots_rl_motion.handlers.previous_action import PreviousActionHandler


class RLNode(Node, ABC):
    """Node to control the wolfgang humanoid."""

    def __init__(self, node_name: str):
        super().__init__(f"{node_name}")

        # Fallback values if parameters don't exist in config file
        self.declare_parameter("model", "")
        self.declare_parameter("phase.control_dt", 0.0)
        self.declare_parameter("phase.gait_frequency", 0.0)
        self.declare_parameter("phase.use_phase", False)
        self.declare_parameter("phase.initial_phase", [0.0, np.pi])
        self.declare_parameter("providers", ["CPUExecutionProvider"])
        self.declare_parameter("joints.ordered_relevant_joint_names", [""])
        self.declare_parameter("joints.walkready_state", [0.0])
        self.declare_parameter(
            "joints.action_scales", [0.5] * len(self.get_parameter("joints.ordered_relevant_joint_names").value)
        )
        self.declare_parameter(
            "joints.kp", [55.0] * len(self.get_parameter("joints.ordered_relevant_joint_names").value)
        )
        self.declare_parameter(
            "joints.kd", [0.6] * len(self.get_parameter("joints.ordered_relevant_joint_names").value)
        )
        # Per-joint sign to convert between the robot's joint convention and the
        # policy's joint convention. Applied to joint pos/vel on read and to the
        # joint target on write. Default +1 (no flip) keeps existing nodes unchanged.
        self.declare_parameter(
            "joints.joint_signs", [1.0] * len(self.get_parameter("joints.ordered_relevant_joint_names").value)
        )
        # Hard joint limits the commanded position is clamped to, shrunk around their
        # center by the soft limit factor. Defaulting to a range no policy can reach
        # keeps the clamp inactive for nodes that do not configure their limits.
        self.declare_parameter(
            "joints.position_limits_lower",
            [-1e6] * len(self.get_parameter("joints.ordered_relevant_joint_names").value),
        )
        self.declare_parameter(
            "joints.position_limits_upper",
            [1e6] * len(self.get_parameter("joints.ordered_relevant_joint_names").value),
        )
        self.declare_parameter("joints.soft_limit_factor", 1.0)
        # Bound on a single action. The action is fed back into the observation of the
        # next step, so an action far outside what the policy produced during training
        # makes the next one worse still, and the loop runs away within a few steps.
        # Bounding it keeps a bad observation from escalating into a diverging one.
        self.declare_parameter("joints.action_limit", 1e6)
        # Joints that are observed but excluded from the published JointCommand
        # (left to other controllers, e.g. the head behavior). Default [""]
        # matches no joint, so nothing is excluded.
        self.declare_parameter("joints.uncontrolled_joint_names", [""])
        # Publishes what the policy sees and what it answers, so a policy that misbehaves
        # on the robot can be compared against the distribution it was trained on
        self.declare_parameter("debug.publish_observation", False)

        model = self.get_parameter("model").value
        self.get_logger().info(f"Loaded model: {model}")

        # Phase is optional - if phase shouldn't be used, than self._phase.get_phase() will return None
        self._phase = PhaseHandler(self)
        self._previous_action = PreviousActionHandler(self)

        # Tracks whether the policy was running on the previous step so that the
        # transition from inactive -> active can be detected and the observation
        # state (re)initialized exactly once per activation.
        self._policy_active = False

        self._observation_publisher = None
        self._action_publisher = None
        if self.get_parameter("debug.publish_observation").value:
            self._observation_publisher = self.create_publisher(Float32MultiArray, f"debug/{node_name}/observation", 1)
            self._action_publisher = self.create_publisher(Float32MultiArray, f"debug/{node_name}/action", 1)

    def _timer_callback(self):
        # Check whether all subscribers received at least one message

        sensors_ready, missing_handler = self._all_sensors_ready()
        if not sensors_ready:
            self.get_logger().warning(
                f"Waiting for all sensors to be available. Following handler hasn't got the needed information: {missing_handler}",
                throttle_duration_sec=10.0,
            )
            return

        # Only run the policy while it is in an allowed (active) state. When it is
        # not active nothing is observed, inferred or published; the next
        # activation starts from a clean observation state.
        if not self.allowed_states():
            self._policy_active = False
            return

        # First step of a (re)activation: let the node initialize its observation
        # state (e.g. fill/clear history buffers) before any inference runs, so
        # the history does not start saturated with stale pre-activation data.
        if not self._policy_active:
            self.initialize_observation()
            self._policy_active = True

        # TODO consider IMU mounting offset

        if self._phase.check_phase_set():
            self._phase.set_obs_phase(
                np.array(
                    [np.cos(self._phase.get_phase()), np.sin(self._phase.get_phase())],
                    dtype=np.float32,
                ).flatten()
            )

        observation = self.obs()

        # Guard against non-finite observations. A single NaN/inf would be fed
        # through the network and, via the previous-action term, poison the
        # feedback loop for every following step. Skip without updating
        # previous_action and report where it is so the source can be found.
        if not np.all(np.isfinite(observation)):
            bad = np.where(~np.isfinite(observation))[0]
            self.get_logger().error(
                f"Non-finite observation: {bad.size} value(s), first indices {bad[:8].tolist()}. Skipping inference.",
                throttle_duration_sec=2.0,
            )
            return

        # Run the ONNX model
        onnx_input = {self._onnx_input_name[0]: observation.reshape(1, -1)}
        onnx_pred = self._onnx_session.run(self._onnx_output_name, onnx_input)[0][0]

        # Guard against a non-finite action: do not feed it back into the
        # observation history and do not publish it. Reset the previous action
        # so the loop can recover instead of staying poisoned.
        if not np.all(np.isfinite(onnx_pred)):
            self.get_logger().error(
                "Network produced a non-finite action; skipping publish and resetting previous action.",
                throttle_duration_sec=2.0,
            )
            self._previous_action.set_previous_action(np.zeros_like(onnx_pred))
            return

        # Bound before the action is fed back, so the feedback loop can not run away
        action_limit = self.get_parameter("joints.action_limit").value
        if np.any(np.abs(onnx_pred) > action_limit):
            self.get_logger().warning(
                f"Action of {np.abs(onnx_pred).max():.1f} exceeds the limit of {action_limit}, "
                "which means the policy is seeing something it was not trained on.",
                throttle_duration_sec=2.0,
            )
            onnx_pred = np.clip(onnx_pred, -action_limit, action_limit)

        self._previous_action.set_previous_action(onnx_pred)
        self.publisher(onnx_pred)
        self._publish_debug(observation, onnx_pred)
        self._phase_update_hook()

    def _publish_debug(self, observation: np.ndarray, action: np.ndarray) -> None:
        if self._observation_publisher is None or self._action_publisher is None:
            return
        self._observation_publisher.publish(Float32MultiArray(data=observation.astype(float).tolist()))
        self._action_publisher.publish(Float32MultiArray(data=action.astype(float).tolist()))

    @abstractmethod
    def _phase_update_hook(self) -> None:
        pass

    @abstractmethod
    def initialize_observation(self):
        """Reset/seed the observation state at the start of each activation.

        Called once on every inactive -> active transition, before the first
        inference of that activation. Stateful nodes (e.g. ones with observation
        history buffers) must clear their state here so the history is rebuilt
        from fresh sensor data instead of stale pre-activation values. Stateless
        nodes can implement this as a no-op.
        """
        pass

    def _all_sensors_ready(self) -> tuple[bool, str]:
        for handler in self._handlers:
            if not handler.has_data():
                return False, type(handler).__name__

        return True, "No missing handler"

    def load_model(self, model) -> None:
        path_to_model = os.path.join(get_package_share_directory("bitbots_rl_motion"), "models", model)

        self._onnx_model_path = Path(path_to_model)
        self.get_logger().warning(f"Loading ONNX model from {self._onnx_model_path}")
        # Load the ONNX model
        self._onnx_session = rt.InferenceSession(self._onnx_model_path, providers=self.get_parameter("providers").value)

        self._onnx_input_name = [inp.name for inp in self._onnx_session.get_inputs()]
        self._onnx_output_name = [out.name for out in self._onnx_session.get_outputs()]

        self._handlers = []

        for _, value in self.__dict__.items():
            if isinstance(value, Handler):
                self._handlers.append(value)

        self._timer = self.create_timer(self.get_parameter("phase.control_dt").value, self._timer_callback)

    @abstractmethod
    def publisher(self, action: np.ndarray) -> None:
        pass

    @abstractmethod
    def obs(self) -> np.ndarray:
        pass

    @abstractmethod
    def allowed_states(self) -> bool:
        pass


def create_main(input_node):
    def main():
        rclpy.init()

        node = input_node()

        executor = EventsExecutor()
        executor.add_node(node)

        try:
            executor.spin()
        finally:
            node.destroy_node()
            rclpy.shutdown()

    return main
