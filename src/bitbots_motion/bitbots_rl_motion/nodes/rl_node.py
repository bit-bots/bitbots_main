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
from pathlib import Path

from abc import ABC, abstractmethod
import numpy as np
import onnx
import onnxruntime as rt
import yaml
from ament_index_python import get_package_share_directory
from bitbots_rl_motion.phase import PhaseObject
from rclpy.node import Node
from rclpy.subscription import Subscription

from handlers.handler import Handler


class RLNode(Node, ABC):
    """Node to control the wolfgang humanoid."""

    def __init__(self, config_path: str, node_name: str):
        super().__init__(f"{node_name}")
        
        self._config = self._load_config(config_path)
        self._phase = PhaseObject(self._config)

    def _load_config(self, path: str):
        with open(path) as f:
            return yaml.safe_load(f)

    def _timer_callback(self):
        if not self._config:
            raise ConfigError("Configuration is missing!")

        # Prüfen ob alle Subscriber schon mindestens eine Nachricht hatten
        if not self._all_sensors_ready():
            self.get_logger().warning("Waiting for all sensors to be available", throttle_duration_sec=1.0)

        # TODO consider IMU mounting offset

        self._phase.set_phase(
            np.array(
                [np.cos(self._phase.get_phase()), np.sin(self._phase.get_phase())],
                dtype=np.float32,
            ).flatten()
        )

        # Run the ONNX model
        onnx_input = {self._onnx_input_name[0]: self.obs().reshape(1, -1)}  # TODO: Improve input
        onnx_pred = self._onnx_session.run(self._onnx_output_name, onnx_input)[0][0]
        self._previous_action = onnx_pred

        self.publisher(onnx_pred)

        phase_tp1 = self._phase.get_phase() + self._phase.get_phase_dt()
        self._phase.set_phase(np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi)

        
    def _all_sensors_ready(self):
        for handler in self._handlers:
            if not handler.has_data():
                return False
            
        return True

    def load_model(self, model):
        path_to_model = os.path.join(get_package_share_directory("bitbots_rl_motion"), "models", model)

        self._onnx_model_path = Path(path_to_model)

        # Load the ONNX model
        self._onnx_session = rt.InferenceSession(self._onnx_model_path, providers=self._config["providers"])
        self._onnx_model = onnx.load(self._onnx_model_path)

        self._onnx_input_name = [inp.name for inp in self._onnx_model.graph.input]
        self._onnx_output_name = [out.name for out in self._onnx_model.graph.output]

        self._subs = []
        self._handlers = []

        for key, value in self.__dict__.items():
            if isinstance(value, Subscription):
                self._subs.append(value)
            if isinstance(value, Handler):
                self._handlers.append(value)
        
        self._timer = self.create_timer(self._config["phase"]["control_dt"], self._timer_callback)

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

class ConfigError(Exception):
    pass
