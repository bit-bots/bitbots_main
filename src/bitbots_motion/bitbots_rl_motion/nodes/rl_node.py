import os
from abc import ABC, abstractmethod
from pathlib import Path

import numpy as np
import onnxruntime as rt
import rclpy
from ament_index_python import get_package_share_directory
from bitbots_rl_motion.phase import Phase
from bitbots_rl_motion.previous_action import PreviousAction
from handlers.handler import Handler
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node


class RLNode(Node, ABC):
    """Node to control the wolfgang humanoid."""

    def __init__(self, node_name: str):
        super().__init__(f"{node_name}")

        # Fallback values if paramters don't exist in config file
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
        # Joints that are observed but excluded from the published JointCommand
        # (left to other controllers, e.g. the head behavior). Default [""]
        # matches no joint, so nothing is excluded.
        self.declare_parameter("joints.uncontrolled_joint_names", [""])

        model = self.get_parameter("model").value
        self.get_logger().info(f"Loaded model: {model}")

        # Phase is optional - if phase shouldn't be used, than self._phase.get_phase() will return None
        self._phase = Phase(self)
        self._previous_action = PreviousAction(self)

        # Tracks whether the policy was running on the previous step so that the
        # transition from inactive -> active can be detected and the observation
        # state (re)initialized exactly once per activation.
        self._policy_active = False

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

        self._previous_action.set_previous_action(onnx_pred)
        self.publisher(onnx_pred)
        self._phase_update_hook()

    @abstractmethod
    def _phase_update_hook(self):
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

    def _all_sensors_ready(self):
        for handler in self._handlers:
            if not handler.has_data():
                return False, type(handler).__name__

        return True, "No missing handler"

    def load_model(self, model):
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
    def publisher(self, action):
        pass

    @abstractmethod
    def obs(self):
        pass

    @abstractmethod
    def allowed_states(self):
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
