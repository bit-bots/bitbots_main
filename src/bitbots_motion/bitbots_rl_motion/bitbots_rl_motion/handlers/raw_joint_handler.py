from typing import Optional

import numpy as np
from sensor_msgs.msg import JointState

from bitbots_msgs.msg import JointCommand
from bitbots_rl_motion.handlers import Handler


class RawJointHandler(Handler):
    """Joint handler for HoST-style policies.

    Differs from :class:`bitbots_rl_motion.handlers.joint_handler.JointHandler`
    in the two aspects that are HoST-specific:

    * ``get_raw_angle_data`` returns absolute joint positions. HoST builds its
      observation from raw ``dof_pos``, without subtracting a default pose.
    * ``get_joint_commands`` implements the HoST control law
      ``q_target = q_current + action * action_scale``
      (``host_ground.py:_compute_torques``), with a single scalar scale instead
      of the per-joint ``joints.action_scales``.
    """

    def __init__(self, node, action_scale: float):
        self._node = node
        self._action_scale = float(action_scale)

        self._ordered_relevant_joint_names = self._node.get_parameter("joints.ordered_relevant_joint_names").value

        self._joint_state: Optional[JointState] = None
        self._joint_state_indices: Optional[list[int]] = None

        self._joint_state_sub = self._node.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )

        num_joints = len(self._ordered_relevant_joint_names)
        self._joint_command = JointCommand()
        self._joint_command.joint_names = list(self._ordered_relevant_joint_names)
        self._joint_command.velocities = [-1.0] * num_joints
        self._joint_command.accelerations = [-1.0] * num_joints
        self._joint_command.max_torques = [-1.0] * num_joints
        self._joint_command.kp = np.array(self._node.get_parameter("joints.kp").value, dtype=np.float64)
        self._joint_command.kd = np.array(self._node.get_parameter("joints.kd").value, dtype=np.float64)
        for name, gains in (("joints.kp", self._joint_command.kp), ("joints.kd", self._joint_command.kd)):
            assert len(gains) == num_joints, f"{name} has {len(gains)} entries but {num_joints} joints are configured"

    def _joint_state_callback(self, msg: JointState) -> None:
        if self._joint_state is not None and self._joint_state.name != msg.name:
            self._joint_state_indices = None
        self._joint_state = msg

    def has_data(self) -> bool:
        return self._joint_state is not None

    def get_raw_angle_data(self) -> np.ndarray:
        joint_state, indices = self._state_and_indices()
        return np.array([joint_state.position[idx] for idx in indices], dtype=np.float32)

    def get_velocity_data(self) -> np.ndarray:
        joint_state, indices = self._state_and_indices()
        return np.array([joint_state.velocity[idx] for idx in indices], dtype=np.float32)

    def get_joint_commands(self, onnx_pred: np.ndarray) -> JointCommand:
        joint_state, _ = self._state_and_indices()

        q_target = self.get_raw_angle_data() + onnx_pred.astype(np.float32) * self._action_scale

        self._joint_command.header.stamp = joint_state.header.stamp
        self._joint_command.positions = q_target.tolist()
        return self._joint_command

    def _state_and_indices(self) -> tuple[JointState, list[int]]:
        assert self._joint_state is not None, "No joint state received yet"
        if self._joint_state_indices is None:
            self._joint_state_indices = [
                self._joint_state.name.index(name) for name in self._ordered_relevant_joint_names
            ]
        return self._joint_state, self._joint_state_indices
