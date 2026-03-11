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

from pathlib import Path
from typing import Callable, NamedTuple

import numpy as np
import onnx
import onnxruntime as rt
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription

CONTROL_DT = 0.02  # Control loop frequency in seconds
GAIT_FREQUENCY = 1.5  # Gait frequency in Hz


class RLNode(Node):
    """Node to control the wolfgang humanoid."""

    # TODO: _previous_action: np.ndarray = np.zeros(len(ORDERED_RELEVANT_JOINT_NAMES), dtype=np.float32)
    _phase: np.ndarray = np.array([0.0, np.pi], dtype=np.float32)
    _phase_dt: float

    class PublisherParam(NamedTuple):
        msg_type: int
        topic: str
        qos_profile: int | QoSProfile

    class SubscriptionParam(NamedTuple):
        msg_type: int
        topic: str
        callback: Callable
        qos_profile: int | QoSProfile

    def __init__(self, path_to_model, handlers):
        self._onnx_model_path = Path(path_to_model)
        model_name = self._onnx_model_path.stem
        super().__init__(f"{model_name}")

        # Load the ONNX model
        self._onnx_session = rt.InferenceSession(self._onnx_model_path, providers=["CPUExecutionProvider"])
        self._onnx_model = onnx.load(self._onnx_model_path)

        self._onnx_input_name = []
        for inp in self._onnx_model.graph.input:
            self._onnx_input_name.append(inp)

        self._onnx_output_name = []
        for out in self._onnx_model.graph.output:
            self._onnx_output_name.append(out)

        # Phase time
        self._phase_dt = 2 * np.pi * GAIT_FREQUENCY * CONTROL_DT

        self._timer = self.create_timer(CONTROL_DT, self._timer_callback)

        self.load_phase()

        self._subs = []

        for key, value in self.__dict__.values():
            if type(value) is Subscription:
                self._subs.append(key)

    # TODO: fix
    def _timer_callback(self):
        for subscription in self._subs:
            if subscription is None:
                self.get_logger().warning("Waiting for all sensors to be available", throttle_duration_sec=1.0)

                for subscription in self._subs:
                    if subscription is None:
                        self.get_logger().warning(f"Waiting for: {subscription} to be available", throttle_duration=1.0)

                return

        # TODO consider IMU mounting offset

        self._obs_phase = np.array([np.cos(self._phase), np.sin(self._phase)], dtype=np.float32).flatten()

        # Run the ONNX model
        onnx_input = {self._onnx_input_name[0]: self.obs().reshape(1, -1)}  # TODO: Improve input
        onnx_pred = self._onnx_session.run(self._onnx_output_name, onnx_input)[0][0]
        self._previous_action = onnx_pred

        self.publisher(onnx_pred)

        phase_tp1 = self._phase + self._phase_dt
        self._phase = np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi

    def obs():
        # Should be defined in subclass
        pass

    def publisher():
        # Should be defined in subclass
        pass

    def load_phase():
        # Should be defined in subclass
        pass


def main():
    import rclpy

    rclpy.init()
    node = RLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()
