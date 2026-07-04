from typing import Optional

import numpy as np
from sensor_msgs.msg import JointState

from bitbots_msgs.msg import JointCommand
from bitbots_rl_motion.handlers import Handler


class JointHandler(Handler):
    def __init__(self, node):
        self._node = node

        self._ordered_relevant_joint_names = self._node.get_parameter("joints.ordered_relevant_joint_names").value
        self._walkready_state = np.array(self._node.get_parameter("joints.walkready_state").value, dtype=np.float32)
        self._action_scales = np.array(self._node.get_parameter("joints.action_scales").value, dtype=np.float32)
        self._joint_signs = np.array(self._node.get_parameter("joints.joint_signs").value, dtype=np.float32)
        self._kp = np.array(self._node.get_parameter("joints.kp").value, dtype=np.float32)
        self._kd = np.array(self._node.get_parameter("joints.kd").value, dtype=np.float32)
        self._previous_action: np.ndarray = np.zeros(len(self._ordered_relevant_joint_names), dtype=np.float32)
        self._joint_state: Optional[JointState] = None

        # Joints that are observed but not published (left to other controllers).
        # Their indices are dropped from the published JointCommand only; the
        # observation still reads every joint in ordered_relevant_joint_names.
        uncontrolled = set(self._node.get_parameter("joints.uncontrolled_joint_names").value)
        self._publish_indices = [
            i for i, name in enumerate(self._ordered_relevant_joint_names) if name not in uncontrolled
        ]
        published_joint_names = [self._ordered_relevant_joint_names[i] for i in self._publish_indices]

        self._joint_state_sub = self._node.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )

        # Latest joint command actually being executed by the robot (combined
        # motor goals published by the HCM). Used to reconstruct what action the
        # policy would have produced while it is not running, so the action
        # history can be kept warm. Keyed by joint name; a message may carry only
        # a subset of joints (e.g. head-only), so entries persist across updates.
        self._latest_command: dict[str, float] = {}
        self._joint_command_sub = self._node.create_subscription(
            JointCommand, "joint_command", self._joint_command_callback, 10
        )
        self._joint_command = JointCommand()
        self._joint_command.joint_names = published_joint_names

        self._joint_command.velocities = [-1.0] * len(published_joint_names)
        self._joint_command.accelerations = [-1.0] * len(published_joint_names)
        # self._joint_command.max_torques = [-1.0] * len(published_joint_names)
        self._joint_command.kp = self._kp[self._publish_indices]
        self._joint_command.kd = self._kd[self._publish_indices]

        self._joint_state_indices = None

    def _joint_state_callback(self, msg: JointState):
        if self._joint_state is not None and self._joint_state.name != msg.name:
            self._joint_state_indices = None
        self._joint_state = msg

    def _joint_command_callback(self, msg: JointCommand):
        for name, position in zip(msg.joint_names, msg.positions):
            self._latest_command[name] = position

    def has_data(self):
        return self._joint_state is not None

    def get_angle_data(self) -> np.ndarray:
        if self._joint_state_indices is None:
            self._cache_joint_state_indices()

        assert self._joint_state is not None
        assert self._joint_state_indices is not None

        joint_angles = (
            np.array(
                [self._joint_state.position[idx] for idx in self._joint_state_indices],
                dtype=np.float32,
            )
            * self._joint_signs
            - self._walkready_state
        )

        return joint_angles

    def get_velocity_data(self) -> np.ndarray:
        if self._joint_state_indices is None:
            self._cache_joint_state_indices()

        assert self._joint_state is not None
        assert self._joint_state_indices is not None

        joint_velocities = (
            np.array(
                [self._joint_state.velocity[idx] for idx in self._joint_state_indices],
                dtype=np.float32,
            )
            * self._joint_signs
        )
        return joint_velocities

    def get_joint_commands(self, onnx_pred, relative_to_current: bool = False) -> JointCommand:
        if self._joint_state_indices is None:
            self._cache_joint_state_indices()

        assert self._joint_state is not None
        assert self._joint_state_indices is not None

        self._joint_command.header.stamp = self._joint_state.header.stamp  # self._node.get_clock().now().to_msg()

        if relative_to_current:
            # Relative-to-current action space (mjlab RelativeJointPositionAction):
            # target = current_joint_position + action * scale.
            # Used by the getup policy. Do NOT offset by the default pose here.
            current = np.array(
                [self._joint_state.position[idx] for idx in self._joint_state_indices],
                dtype=np.float32,
            )
            positions = onnx_pred * self._action_scales + current
        else:
            # Default-pose-relative action space (e.g. the walk policy):
            # target = default_pose + action * scale.
            # Target is built in the policy's joint convention, then converted back to
            # the robot's convention with joint_signs (inverse of the read mapping).
            positions = (onnx_pred * self._action_scales + self._walkready_state) * self._joint_signs
        # Uncontrolled joints (e.g. the head) are dropped from the published command.
        self._joint_command.positions = positions[self._publish_indices]
        return self._joint_command

    def reconstruct_previous_action(self) -> np.ndarray:
        """Infer the action that would have produced the currently executed joint command.

        While the policy is not running, its action-history term has no real
        previous action to feed back. This reconstructs a plausible one by
        inverting the default-pose-relative mapping used in ``get_joint_commands``
        (``positions = (action * action_scales + walkready_state) * joint_signs``)
        applied to the joint command the robot is actually executing:

            ``action = (positions * joint_signs - walkready_state) / action_scales``

        (``joint_signs`` is +-1, so multiplying by it inverts it.) Joints for
        which no command has been observed yet fall back to the action that
        reproduces the measured joint angle, i.e. "hold the current pose".
        """
        # Fallback per joint: action reproducing the measured joint angle.
        # get_angle_data() already returns measured * joint_signs - walkready_state.
        action = self.get_angle_data() / self._action_scales
        for i, name in enumerate(self._ordered_relevant_joint_names):
            position = self._latest_command.get(name)
            if position is not None:
                action[i] = (position * self._joint_signs[i] - self._walkready_state[i]) / self._action_scales[i]
        return action.astype(np.float32)

    def get_previous_action_initial(self) -> np.ndarray:
        return self._previous_action

    def _cache_joint_state_indices(self):
        assert self._joint_state is not None
        self._joint_state_indices = [self._joint_state.name.index(name) for name in self._ordered_relevant_joint_names]
