from typing import Optional

import numpy as np
from sensor_msgs.msg import JointState

from bitbots_msgs.msg import JointCommand
from handlers.handler import Handler


class JointHandler(Handler):
    def __init__(self, node):
        self._node = node

        self._ordered_relevant_joint_names = self._node.get_parameter("joints.ordered_relevant_joint_names").value
        self._walkready_state = self._node.get_parameter("joints.walkready_state").value
        self._action_scales = np.array(
            self._node.get_parameter("joints.action_scales").value, dtype=np.float32
        )
        self._kp = np.array(
            self._node.get_parameter("joints.kp").value, dtype=np.float32
        )
        self._kd = np.array(
            self._node.get_parameter("joints.kd").value, dtype=np.float32
        )
        self._previous_action: np.ndarray = np.zeros(len(self._ordered_relevant_joint_names), dtype=np.float32)
        self._joint_state: Optional[JointState] = None

        self._joint_state_sub = self._node.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )
        self._joint_command = JointCommand()
        self._joint_command.joint_names = self._ordered_relevant_joint_names

        self._joint_command.velocities = [-1.0] * len(self._ordered_relevant_joint_names)
        self._joint_command.accelerations = [-1.0] * len(self._ordered_relevant_joint_names)
        #self._joint_command.max_torques = [-1.0] * len(self._ordered_relevant_joint_names)
        self._joint_command.kp = self._kp
        self._joint_command.kd = self._kd

        self._joint_state_indices = None


    def _joint_state_callback(self, msg):
        self._joint_state = msg

    def has_data(self):
        return self._joint_state is not None

    def get_angle_data(self) -> np.ndarray:
        assert self._joint_state is not None
        if self._joint_state_indices is None:
            self._cache_joint_state_indices()
        joint_angles = (
            np.array(
                [
                    self._joint_state.position[idx]
                    for idx in self._joint_state_indices
                ],
                dtype=np.float32,
            )
            - self._walkready_state
        )

        return joint_angles

    def get_velocity_data(self) -> np.ndarray:
        assert self._joint_state is not None
        if self._joint_state_indices is None:
            self._cache_joint_state_indices()
        joint_velocities = (
            np.array(
                [
                    self._joint_state.velocity[idx]
                    for idx in self._joint_state_indices
                ],
                dtype=np.float32,
            )
        )
        return joint_velocities

    def get_joint_commands(self, onnx_pred) -> JointCommand:
        
        self._joint_command.header.stamp = self._joint_state.header.stamp #self._node.get_clock().now().to_msg()
        self._joint_command.positions = onnx_pred * self._action_scales + self._walkready_state
        return self._joint_command

    def get_previous_action_initial(self) -> np.ndarray:
        return self._previous_action

    def _cache_joint_state_indices(self):
        self._joint_state_indices = [
            self._joint_state.name.index(name) for name in self._ordered_relevant_joint_names
        ]
