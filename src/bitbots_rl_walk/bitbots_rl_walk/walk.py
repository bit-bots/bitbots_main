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
import os

from typing import Optional
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from sensor_msgs.msg import JointState, Imu
from bitbots_msgs.msg import JointCommand
from geometry_msgs.msg import Twist, PoseStamped, Pose
from transforms3d.quaternions import quat2mat
from transforms3d.euler import euler2mat
from ros2_numpy import numpify

import numpy as np
import onnxruntime as rt
from ament_index_python import get_package_share_directory

ONNX_MODEL = os.path.join(
    get_package_share_directory("bitbots_rl_walk"), "models", "piplus_policy_fixed_joint_axis.onnx"
)

WALKREADY_STATE = np.array([
    0.6, 0.0, 0.0,  1.2,  0.6, 0,
   -0.6, 0.0, 0.0, -1.2, -0.6, 0
], dtype=np.float32)

CONTROL_DT = 0.02  # Control loop frequency in seconds

GAIT_FREQUENCY = 1.5  # Gait frequency in Hz

ORDERED_RELEVANT_JOINT_NAMES = [
    "r_hip_pitch_joint",
    "r_hip_roll_joint",
    "r_thigh_joint",
    "r_calf_joint",
    "r_ankle_pitch_joint",
    "r_ankle_roll_joint",
    "l_hip_pitch_joint",
    "l_hip_roll_joint",
    "l_thigh_joint",
    "l_calf_joint",
    "l_ankle_pitch_joint",
    "l_ankle_roll_joint",
]


class WalkNode(Node):
    """Node to control the wolfgang humanoid."""
    _previous_action: np.ndarray = np.zeros(
        len(ORDERED_RELEVANT_JOINT_NAMES), dtype=np.float32
    )
    _imu_data: Optional[Imu] = None
    _joint_state: Optional[JointState] = None
    _cmd_vel: Optional[Twist] = None
    _phase: np.ndarray = np.array([0.0, np.pi], dtype=np.float32)
    _phase_dt: float

    def __init__(self):
        super().__init__("reinforcement_learning_walk_inference_node")

        # Set sim time parameter to true
        #self.set_parameters([
        #    Parameter('use_sim_time', Parameter.Type.BOOL, True), ])

        self._phase_dt = 2 * np.pi * GAIT_FREQUENCY * CONTROL_DT

        # Load the ONNX model
        self._onnx_session = rt.InferenceSession(
            ONNX_MODEL,
            providers=["CPUExecutionProvider"]
        )

        self._joint_command_pub = self.create_publisher(
            JointCommand, "walking_motor_goals", 10
        )
        self._imu_sub = self.create_subscription(
            Imu, "imu/data", self._imu_callback, 10
        )
        self._joint_state_sub = self.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )
        self._cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self._cmd_vel_callback, 10
        )

        self._timer = self.create_timer(CONTROL_DT, self._timer_callback)


    def _joint_state_callback(self, msg: JointState):
        self._joint_state = msg

    def _cmd_vel_callback(self, msg: Twist):
        self._cmd_vel = msg

    def _imu_callback(self, msg: Imu):
        self._imu_data = msg

    def _timer_callback(self):
        """Timer callback to publish joint commands based on the ONNX policy."""
        if self._imu_data is None or self._joint_state is None or self._cmd_vel is None:
            self.get_logger().warning(
                "Waiting for all sensors to be available", throttle_duration_sec=1.0
            )

            # Print the sensor that we are still waiting for
            if self._imu_data is None:
                self.get_logger().warning("Waiting for IMU data", throttle_duration_sec=1.0)
            if self._joint_state is None:
                self.get_logger().warning("Waiting for joint state data", throttle_duration_sec=1.0)
            if self._cmd_vel is None:
                self.get_logger().warning("Waiting for cmd_vel data", throttle_duration_sec=1.0)

            return

        # Prepare the observation vector
        gyro = np.array([
            self._imu_data.angular_velocity.x,
            self._imu_data.angular_velocity.y,
            self._imu_data.angular_velocity.z,
        ], dtype=np.float32)

        gravity = (quat2mat(
            [self._imu_data.orientation.w,
             self._imu_data.orientation.x,
             self._imu_data.orientation.y,
             self._imu_data.orientation.z]
        ) @ euler2mat(0, 0.0, 0)).T @ np.array([0, 0, -1], dtype=np.float32)

        joint_angles = np.array([
            self._joint_state.position[self._joint_state.name.index(name)]
            for name in ORDERED_RELEVANT_JOINT_NAMES
        ], dtype=np.float32) - WALKREADY_STATE

        joint_velocities = np.array([
            self._joint_state.velocity[self._joint_state.name.index(name)]
            for name in ORDERED_RELEVANT_JOINT_NAMES
        ], dtype=np.float32)

        phase = np.array([np.cos(self._phase), np.sin(self._phase)], dtype=np.float32).flatten()

        command = np.array([
            self._cmd_vel.linear.x,
            self._cmd_vel.linear.y,
            self._cmd_vel.angular.z
        ], dtype=np.float32)

        obs = np.hstack([
            gyro,
            gravity,
            command,
            joint_angles,
            joint_velocities,
            self._previous_action,  # Previous action
            phase
        ]).astype(np.float32)

        # Run the ONNX model
        onnx_input = {"obs": obs.reshape(1, -1)}
        onnx_pred = self._onnx_session.run(
            ["continuous_actions"], onnx_input
        )[0][0]
        self._previous_action = onnx_pred

        # Publish the joint commands
        joint_command = JointCommand()
        joint_command.header.stamp = self._joint_state.header.stamp
        joint_command.joint_names = ORDERED_RELEVANT_JOINT_NAMES
        joint_command.positions = onnx_pred * 0.5 + WALKREADY_STATE
        joint_command.velocities = [-1.0] * len(ORDERED_RELEVANT_JOINT_NAMES)
        joint_command.accelerations = [-1.0] * len(ORDERED_RELEVANT_JOINT_NAMES)
        joint_command.kp = [30.0] * len(ORDERED_RELEVANT_JOINT_NAMES)
        joint_command.kd = [1.1] * len(ORDERED_RELEVANT_JOINT_NAMES)


        self._joint_command_pub.publish(joint_command)

        # Stop phase if the robot gets a !=0 cmd_vel.angular.x command, which is used as a stop signal for the walking
        if not (self._cmd_vel.angular.x != 0.0 and \
                abs(self._cmd_vel.linear.x) == 0.0 and \
                abs(self._cmd_vel.linear.y) == 0.0 and \
                abs(self._cmd_vel.angular.z) == 0.0 and \
                np.linalg.norm(self._phase - np.array([0.0, np.pi], dtype=np.float32)) < 0.1):
            phase_tp1 = self._phase + self._phase_dt
            self._phase = np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi


def main():
    import rclpy

    rclpy.init()
    node = WalkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.try_shutdown()
