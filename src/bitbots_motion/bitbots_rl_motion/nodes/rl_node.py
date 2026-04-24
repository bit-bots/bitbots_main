# Copyright 2024 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Deploy an MJX policy in ONNX format to C MuJoCo and play with it."""

import os
from abc import ABC, abstractmethod
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as rt
import rclpy
from ament_index_python import get_package_share_directory
from bitbots_rl_motion.phase import PhaseObject
from bitbots_rl_motion.previous_action import PreviousActionObject
from handlers.handler import Handler
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from rclpy.subscription import Subscription


class RLNode(Node, ABC):
    """Node to control the wolfgang humanoid."""

    def __init__(self, node_name: str):
        super().__init__(f"{node_name}")

        # Fallback values if paramtes don't exist in config file
        self.declare_parameter("model", "")
        self.declare_parameter("phase.control_dt", 0.0)
        self.declare_parameter("phase.gait_frequency", 0.0)
        self.declare_parameter("phase.use_phase", False)
        self.declare_parameter("providers", ["CPUExecutionProvider"])
        self.declare_parameter("joints.ordered_relevant_joint_names", [""])
        self.declare_parameter("joints.walkready_state", [0.0])

        model = self.get_parameter("model").value
        self.get_logger().info(f"Loaded model: {model}")

        # Phase is optional - if phase shouldn't be used, than self._phase.get_phase() will return None
        self._phase = PhaseObject(self)
        self._previous_action = PreviousActionObject(self)

    def _timer_callback(self):
        # Check whether all subscribers hat at least on message

        sensors_ready, missing_handler = self._all_sensors_ready()
        if not sensors_ready:
            self.get_logger().warning(
                f"Waiting for all sensors to be available. Following handler hasn't got the needed information: {missing_handler}",
                throttle_duration_sec=1.0,
            )
            return

        # TODO consider IMU mounting offset

        if self._phase.check_phase_set():
            self._phase.set_obs_phase(
                np.array(
                    [np.cos(self._phase.get_phase()), np.sin(self._phase.get_phase())],
                    dtype=np.float32,
                ).flatten()
            )

        observation = self.obs()

        # Run the ONNX model
        onnx_input = {self._onnx_input_name[0]: observation.reshape(1, -1)}  # TODO: Improve input
        onnx_pred = self._onnx_session.run(self._onnx_output_name, onnx_input)[0][0]
        self._previous_action.set_previous_action(onnx_pred)

        self.publisher(onnx_pred)

        if self._phase.check_phase_set():
            phase_tp1 = self._phase.get_phase() + self._phase.get_phase_dt()
            self._phase.set_phase(np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi)

    def _all_sensors_ready(self):
        for handler in self._handlers:
            if not handler.has_data():
                return False, type(handler).__name__

        return True, "No missing handler"

    def load_model(self, model):
        path_to_model = os.path.join(get_package_share_directory("bitbots_rl_motion"), "models", model)

        self._onnx_model_path = Path(path_to_model)

        # Load the ONNX model
        self._onnx_session = rt.InferenceSession(self._onnx_model_path, providers=self.get_parameter("providers").value)
        self._onnx_model = onnx.load(self._onnx_model_path)

        self._onnx_input_name = [inp.name for inp in self._onnx_model.graph.input]
        self._onnx_output_name = [out.name for out in self._onnx_model.graph.output]

        self._subs = []
        self._handlers = []

        for _, value in self.__dict__.items():
            if isinstance(value, Subscription):
                self._subs.append(value)
            if isinstance(value, Handler):
                self._handlers.append(value)

        self._timer = self.create_timer(self.get_parameter("phase.control_dt").value, self._timer_callback)

        self.load_phase()

    @abstractmethod
    def publisher(self, action):
        pass

    @abstractmethod
    def load_phase(self):
        pass

    @abstractmethod
    def obs(self):
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
