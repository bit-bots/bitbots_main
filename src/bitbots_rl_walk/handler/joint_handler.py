import numpy as np

from bitbots_msgs.msg import JointCommand

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


class JointsHandler:
    def __init__(self, ordered_relevant_joint_names=ORDERED_RELEVANT_JOINT_NAMES, walkready_state=WALKREADY_STATE):
        self._ordered_relevant_joint_names = ordered_relevant_joint_names
        self._walkready_state = walkready_state
        self._joint_state = None

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

    def get_walkready_joints_command(self, timestamp):
        joint_command = JointCommand()
        joint_command.joint_names = self._ordered_relevant_joint_names
        joint_command.velocities = [0.2] * len(self._ordered_relevant_joint_names)
        joint_command.accelerations = [-1.0] * len(self._ordered_relevant_joint_names)
        joint_command.max_currents = [-1.0] * len(self._ordered_relevant_joint_names)  # -1.0 means no limit
        joint_command.header.stamp = timestamp.to_msg()
        joint_command.positions = self._walkready_state

        return joint_command

    def joint_state_callback(self, msg):
        self._joint_state = msg
