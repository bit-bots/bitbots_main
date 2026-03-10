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

import time
from pathlib import Path
from typing import Callable, NamedTuple

import numpy as np
import onnxruntime as rt
from rclpy.node import Node
from rclpy.qos import QoSProfile

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
        self._onnx_model = Path(path_to_model)
        model_name = self._onnx_model.stem
        super().__init__(f"{model_name}")

        # Load the ONNX model
        self._onnx_session = rt.InferenceSession(self._onnx_model, providers=["CPUExecutionProvider"])

        self._pubs = []
        self._subs = {}
        self._states = {}
        self.handlers = handlers

        # Unpack handlers
        # TODO: Handler template!
        for handler in handlers:
            confg = handler.getConfg()
            for sub in confg["subscriber"]:
                self._subs[sub.key] = sub.value
                self._states[f"{sub.key()}"] = None

        # Create subscribers
        self.create_rl_subscriptions(self._subs)

        # Phase time
        # TODO: Check whether handler is necessary
        self._phase_dt = 2 * np.pi * GAIT_FREQUENCY * CONTROL_DT

        self._timer = self.create_timer(CONTROL_DT, self._timer_callback)

        # TODO: self._joint_command_pub.publish(joint_command)
        time.sleep(10)

    def _timer_callback(self):
        for subscription in self._subs:
            if subscription is None:
                self.get_logger().warning("Waiting for all sensors to be available", throttle_duration_sec=1.0)

                for subscription in self._subs:
                    if subscription is None:
                        self.get_logger().warning(f"Waiting for: {subscription} to be available", throttle_duration=1.0)

                return

        # TODO consider IMU mounting offset

        # TODO: is not used! phase = np.array([np.cos(self._phase), np.sin(self._phase)], dtype=np.float32).flatten()

        """
        command = np.array([self._cmd_vel.linear.x, self._cmd_vel.linear.y, self._cmd_vel.angular.z], dtype=np.float32)
        """

        # TODO:
        """
        obs = np.hstack(
            [
                gyro,  # 3
                gravity,  # 4
                command,  # 3
                joint_angles,  # 18
                joint_velocities,  # 18
                self._previous_action,  # 18  # Previous action
                phase,  # 2
            ]
        ).astype(np.float32)
        """

        # Run the ONNX model
        # TODO:
        """
        onnx_input = {"in_0": obs.reshape(1, -1)}
        onnx_pred = self._onnx_session.run(["tanh_out_0"], onnx_input)[0][0]
        self._previous_action = onnx_pred
        """

        """
        # Publish the joint commands
        joint_command = JointCommand()
        joint_command.header.stamp = self._joint_state.header.stamp
        joint_command.joint_names = ORDERED_RELEVANT_JOINT_NAMES
        joint_command.positions = onnx_pred * 0.5 + WALKREADY_STATE
        joint_command.velocities = [-1.0] * len(ORDERED_RELEVANT_JOINT_NAMES)
        joint_command.accelerations = [-1.0] * len(ORDERED_RELEVANT_JOINT_NAMES)
        joint_command.max_currents = [-1.0] * len(ORDERED_RELEVANT_JOINT_NAMES)

        self._joint_command_pub.publish(joint_command)
        """

        phase_tp1 = self._phase + self._phase_dt
        self._phase = np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi


def main():
    import rclpy

    rclpy.init()
    node = RLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()
