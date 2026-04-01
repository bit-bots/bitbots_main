import numpy as np

from bitbots_msgs.msg import JointCommand
from handlers.handler import Handler


class JointHandler(Handler):
    def __init__(self, config):
        super().__init__(config)

        self._ordered_relevant_joint_names = self._config["joints"]["ordered_relevant_joint_names"]
        self._walkready_state = self._config["joints"]["walkready_state"]
        self._previous_action: np.ndarray = np.zeros(len(self._ordered_relevant_joint_names), dtype=np.float32)
        self._joint_state = None

    def joint_state_callback(self, msg):
        self._joint_state = msg

    def has_data(self):
        return self._joint_state is not None

    def get_angle_data(self):
        joint_angles = (
            np.array(
                [
                    self._joint_state.position[self._joint_state.name.index(name)]
                    for name in self._ordered_relevant_joint_names
                ],
                dtype=np.float32,
            )
            - self._walkready_state
        )

        return joint_angles

    def get_velocity_data(self):
        joint_velocities = np.array(
            [
                self._joint_state.velocity[self._joint_state.name.index(name)]
                for name in self._ordered_relevant_joint_names
            ],
            dtype=np.float32,
        )

        return joint_velocities

    def get_data(self):
        return self.get_angle_data(), self.get_velocity_data()

    def get_walkready_joint_command(self, timestamp):
        joint_command = JointCommand()
        joint_command.joint_names = self._ordered_relevant_joint_names
        joint_command.velocities = [0.2] * len(self._ordered_relevant_joint_names)
        joint_command.accelerations = [-1.0] * len(self._ordered_relevant_joint_names)
        joint_command.max_currents = [-1.0] * len(self._ordered_relevant_joint_names)  # -1.0 means no limit
        joint_command.header.stamp = timestamp
        joint_command.positions = self._walkready_state

        return joint_command

    def get_joint_commands(self, onnx_pred):
        joint_command = JointCommand()
        joint_command.header.stamp = self._joint_state.header.stamp
        joint_command.joint_names = self._ordered_relevant_joint_names
        joint_command.positions = onnx_pred * 0.5 + self._walkready_state
        joint_command.velocities = [-1.0] * len(self._ordered_relevant_joint_names)
        joint_command.accelerations = [-1.0] * len(self._ordered_relevant_joint_names)
        joint_command.max_currents = [-1.0] * len(self._ordered_relevant_joint_names)

        return joint_command

    def get_previous_action_initial(self):
        return self._previous_action
