from typing import Optional

import numpy as np
from sensor_msgs.msg import JointState

from bitbots_msgs.msg import JointCommand
from bitbots_rl_motion.handlers import Handler


class JointHandler(Handler):
    def __init__(self, node):
        self._node = node

        self._ordered_relevant_joint_names = self._node.get_parameter("joints.ordered_relevant_joint_names").value
        self._walkready_state = self._node.get_parameter("joints.walkready_state").value
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
        # Names of the joints that actually appear in the published JointCommand
        # (i.e. relevant joints minus the uncontrolled ones), in publish order.
        # Exposed so the walkready transition can build an absolute command over
        # exactly these joints.
        self.published_joint_names = published_joint_names

        self._joint_state_sub = self._node.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
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

    def get_measured_positions(self, names: list[str]) -> np.ndarray:
        """Current measured positions (radians, robot convention) of the given joints.

        Looks the joints up by name in the latest ``joint_states`` message, so the
        returned array is in the same order as ``names``. Used to capture the pose
        the walkready transition should interpolate away from.
        """
        assert self._joint_state is not None
        return np.array(
            [self._joint_state.position[self._joint_state.name.index(name)] for name in names],
            dtype=np.float32,
        )

    def get_absolute_joint_command(self, positions: np.ndarray) -> JointCommand:
        """Build a JointCommand for absolute target positions of the published joints.

        ``positions`` must be in radians in the robot's joint convention and in the
        same order as :attr:`published_joint_names` (no action scaling, joint-sign
        flip or default-pose offset is applied). Unlike :meth:`get_joint_commands`
        this stamps with the current time so downstream consumers keep treating the
        command stream as fresh during the walkready transition.
        """
        self._joint_command.header.stamp = self._node.get_clock().now().to_msg()
        self._joint_command.positions = positions.tolist()
        return self._joint_command

    def get_previous_action_initial(self) -> np.ndarray:
        return self._previous_action

    def _cache_joint_state_indices(self):
        assert self._joint_state is not None
        self._joint_state_indices = [self._joint_state.name.index(name) for name in self._ordered_relevant_joint_names]
