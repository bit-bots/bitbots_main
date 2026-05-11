from typing import Optional

import numpy as np
from sensor_msgs.msg import JointState

from bitbots_msgs.msg import JointCommand
from handlers.handler import Handler


class RawJointHandler(Handler):
    """Joint handler for HoST-style policies.

    Differs from ``JointHandler`` in two aspects that are HoST-specific:

    * ``get_raw_angle_data`` returns absolute joint positions (no walkready
      subtraction). HoST observations are built from raw ``dof_pos``.
    * ``get_joint_commands`` builds a ``JointCommand`` with
      ``q_target = q_current + action * action_scale`` (HoST control law,
      ``host_ground.py:_compute_torques``), not the walking-style
      ``action * scale + walkready_state``.

    The existing ``JointHandler`` is left untouched so that walking and kick
    policies keep their current behaviour.
    """

    def __init__(self, node, action_scale: float):
        self._node = node
        self._action_scale = float(action_scale)

        self._ordered_relevant_joint_names = self._node.get_parameter(
            "joints.ordered_relevant_joint_names"
        ).value

        self._joint_state: Optional[JointState] = None
        self._joint_state_sub = self._node.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        self._joint_state = msg

    def has_data(self) -> bool:
        return self._joint_state is not None

    def get_raw_angle_data(self) -> np.ndarray:
        assert self._joint_state is not None
        return np.array(
            [
                self._joint_state.position[self._joint_state.name.index(name)]
                for name in self._ordered_relevant_joint_names
            ],
            dtype=np.float32,
        )

    def get_velocity_data(self) -> np.ndarray:
        assert self._joint_state is not None
        return np.array(
            [
                self._joint_state.velocity[self._joint_state.name.index(name)]
                for name in self._ordered_relevant_joint_names
            ],
            dtype=np.float32,
        )

    def get_joint_commands(self, onnx_pred: np.ndarray) -> JointCommand:
        assert self._joint_state is not None
        q_current = self.get_raw_angle_data()
        q_target = q_current + onnx_pred.astype(np.float32) * self._action_scale

        joint_command = JointCommand()
        joint_command.header.stamp = self._joint_state.header.stamp
        joint_command.from_hcm = True
        joint_command.joint_names = list(self._ordered_relevant_joint_names)
        joint_command.positions = q_target.tolist()
        joint_command.velocities = [-1.0] * len(self._ordered_relevant_joint_names)
        joint_command.accelerations = [-1.0] * len(self._ordered_relevant_joint_names)
        joint_command.max_torques = [-1.0] * len(self._ordered_relevant_joint_names)
        return joint_command
