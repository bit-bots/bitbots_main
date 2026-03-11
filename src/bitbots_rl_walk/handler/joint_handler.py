import numpy as np

from bitbots_msgs.msg import JointCommand
from bitbots_rl_walk.handler.handler import Handler

ORDERED_RELEVANT_JOINT_NAMES = [
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbow",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbow",
    "RHipYaw",
    "RHipRoll",
    "RHipPitch",
    "RKnee",
    "RAnklePitch",
    "RAnkleRoll",
    "LHipYaw",
    "LHipRoll",
    "LHipPitch",
    "LKnee",
    "LAnklePitch",
    "LAnkleRoll",
]

WALKREADY_STATE = np.array(
    [
        0,
        0,
        0,
        0,
        0,
        0,
        0.023628265148262724,
        -0.10401795710581162,
        -0.7352626990449959,
        -1.3228415184260092,
        0.5495038397740458,
        -0.12913515511895796,
        -0.016441795868928723,
        0.07253788412595062,
        0.7420808433462046,
        1.334527650998329,
        -0.5537397918567754,
        0.07437380704149316,
    ],
    dtype=np.float32,
)


class JointHandler(Handler):
    def __init__(self, ordered_relevant_joint_names=ORDERED_RELEVANT_JOINT_NAMES, walkready_state=WALKREADY_STATE):
        self._ordered_relevant_joint_names = ordered_relevant_joint_names
        self._walkready_state = walkready_state
        self._previous_action: np.ndarray = np.zeros(len(self._ordered_relevant_joint_names), dtype=np.float32)
        self._joint_state = None
        self._obs_phase = None
        self._phase = None

    def set_obs_phase(self, phase):
        self._obs_phase = phase

    def set_phase(self, phase):
        self._phase = phase

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
        joint_command.header.stamp = timestamp.to_msg()
        joint_command.positions = self._walkready_state

        self._previous_action = joint_command

        return joint_command

    def get_joint_commands(self, onnx_pred):
        joint_command = JointCommand()
        joint_command.header.stamp = self._joint_state.header.stamp
        joint_command.joint_names = self._ordered_relevant_joint_names
        joint_command.positions = onnx_pred * 0.5 + self._walkready_state
        joint_command.velocities = [-1.0] * len(self._ordered_relevant_joint_names)
        joint_command.accelerations = [-1.0] * len(self._ordered_relevant_joint_names)
        joint_command.max_currents = [-1.0] * len(self._ordered_relevant_joint_names)

        self._previous_action = joint_command

        return joint_command

    def get_previous_action(self):
        return self._previous_action

    def get_obs_phase(self):
        return self._obs_phase

    def get_phase(self):
        return self._phase

    def joint_state_callback(self, msg):
        self._joint_state = msg
