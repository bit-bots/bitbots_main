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
from std_msgs.msg import Float32MultiArray


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
        self.declare_parameter("gravity.roll_offset", 0.0)
        self.declare_parameter("gravity.pitch_offset", 0.0)
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
        self.declare_parameter("command.max_linear_velocity_forward", 999.0)
        self.declare_parameter("command.max_linear_velocity_backward", -999.0)
        self.declare_parameter("command.max_linear_velocity_sideways", 999.0)
        self.declare_parameter("command.max_angular_velocity", 999.0)

        model = self.get_parameter("model").value
        self.get_logger().info(f"Loaded model: {model}")

        # Phase is optional - if phase shouldn't be used, than self._phase.get_phase() will return None
        self._phase = Phase(self)
        self._phase_init_hook()

        self._previous_action = PreviousAction(self)

    def _timer_callback(self):
        # Check whether all subscribers received at least one message

        sensors_ready, missing_handler = self._all_sensors_ready()
        if not sensors_ready:
            self.get_logger().warning(
                f"Waiting for all sensors to be available. Following handler hasn't got the needed information: {missing_handler}",
                throttle_duration_sec=10.0,
            )
            return

        observation = self.obs()

        # Run the ONNX model
        onnx_input = {self._onnx_input_name[0]: observation.reshape(1, -1)}
        onnx_pred = self._onnx_session.run(self._onnx_output_name, onnx_input)[0][0]
        self._previous_action.set_previous_action(onnx_pred)

        if self.allowed_states():
            self.publisher(onnx_pred)
        self._phase_update_hook()

    @abstractmethod
    def _phase_init_hook(self):
        pass

    @abstractmethod
    def _phase_update_hook(self):
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
