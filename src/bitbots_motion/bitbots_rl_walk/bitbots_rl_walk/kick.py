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

import os
from typing import Optional

import numpy as np
import onnxruntime as rt
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty
from transforms3d.euler import euler2mat
from transforms3d.quaternions import quat2mat
from rclpy.experimental.events_executor import EventsExecutor
from bitbots_tf_buffer import Buffer
from std_msgs.msg import Float32
import tf2_geometry_msgs
import tf2_ros as tf2
from ros2_numpy import numpify
from tf_transformations import euler_from_quaternion
from rclpy.time import Time

from bitbots_msgs.msg import JointCommand

ONNX_MODEL = os.path.join(get_package_share_directory("bitbots_rl_walk"), "models", "policy_kick.onnx")

WALKREADY_STATE = np.array([0.6, 0.0, 0.0, 1.2, 0.6, 0, -0.6, 0.0, 0.0, -1.2, -0.6, 0], dtype=np.float32)

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


class KickNode(Node):
    """Node to control the wolfgang humanoid."""

    _previous_action: np.ndarray = np.zeros(len(ORDERED_RELEVANT_JOINT_NAMES), dtype=np.float32)
    _imu_data: Optional[Imu] = None
    _joint_state: Optional[JointState] = None
    _ball_relative: Optional[PointStamped] = None
    _phase: np.ndarray = np.array([np.pi / 2, -np.pi / 2], dtype=np.float32)
    _phase_dt: float
    _kick_direction: Optional[PoseStamped] = None
    _last_global_phase: Optional[float] = None
    _last_point = None

    def __init__(self):
        super().__init__("reinforcement_learning_kick_inference_node")

        self._phase_dt = 2 * np.pi * GAIT_FREQUENCY * CONTROL_DT

        # Load the ONNX model
        self._onnx_session = rt.InferenceSession(ONNX_MODEL, providers=["CPUExecutionProvider"])

        self._joint_command_pub = self.create_publisher(JointCommand, "walking_motor_goals", 10)
        self._imu_sub = self.create_subscription(Imu, "imu/data", self._imu_callback, 10)
        self._joint_state_sub = self.create_subscription(JointState, "joint_states", self._joint_state_callback, 10)
        self._kick_direction_sub = self.create_subscription(PoseStamped, "kick_direction", self._kick_direction_callback, 10)
        self._kick_stop_pub = self.create_subscription(Empty, "kick_stop", self._kick_stop_callback, 10)
        self._global_phase_sub = self.create_subscription(Float32, "rl_phase", self._other_policy_phase_callback, 10)
        self._global_phase_pub = self.create_publisher(Float32, "rl_phase", 10)

        self.tf_buffer = Buffer(Duration(seconds=1.0), self)

        # Sub to the filtered ball position
        self._ball_sub = self.create_subscription(PoseWithCovarianceStamped, "ball_position_relative_filtered", self._ball_callback, 10)

        self._timer = self.create_timer(CONTROL_DT, self._timer_callback)

    def _kick_direction_callback(self, msg: PoseStamped):
        if self._kick_direction is None and self._last_global_phase is not None:
            self._phase = np.array([self._last_global_phase, (self._last_global_phase + 2 * np.pi) % (2 * np.pi) - np.pi], dtype=np.float32)
        self._kick_direction = msg

    def _kick_stop_callback(self, _: Empty):
        self._kick_direction = None

    def _joint_state_callback(self, msg: JointState):
        self._joint_state = msg

    def _other_policy_phase_callback(self, msg: Float32):
        self._last_global_phase = msg.data

    def _ball_callback(self, ball: PoseWithCovarianceStamped):
        # Transform the ball position to the base_footprint frame
        point = PointStamped()
        point.header.frame_id = ball.header.frame_id
        point.point = ball.pose.pose.position
        try:
            self._ball_relative = self.tf_buffer.transform(point, "odom")
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self._ball_relative = None
            self.get_logger().warn(str(e))

    def _imu_callback(self, msg: Imu):
        self._imu_data = msg

    def _timer_callback(self):
        """Timer callback to publish joint commands based on the ONNX policy."""
        if self._imu_data is None or self._joint_state is None or self._ball_relative is None or self._kick_direction is None:

            # Print the sensor that we are still waiting for
            if self._imu_data is None:
                self.get_logger().warning("Waiting for IMU data", throttle_duration_sec=1.0)
            if self._joint_state is None:
                self.get_logger().warning("Waiting for joint state data", throttle_duration_sec=1.0)

            return

        # Transform direction
        try:
            target_local = self.tf_buffer.transform(self._kick_direction, "base_link")
            yaw = euler_from_quaternion(numpify(target_local.pose.orientation))[2]
            kick_dir = np.array([np.cos(yaw), np.sin(yaw)])
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.get_logger().error("Dropped policy step")
            kick_dir = np.array([1.0, 0.0])
        kick_dir /= np.linalg.norm(kick_dir)

        # Transform ball
        try:
            point_now = PointStamped()
            point_now.header.frame_id = "odom"
            point_now.point = self._ball_relative.point
            point_now.point.z = 0.075
            ball_pose = self.tf_buffer.transform(point_now, "base_link")
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            self.get_logger().error("Dropped policy step")
            self.get_logger().error(str(e))
            return


        # Prepare the observation vector
        gyro = np.array(
            [
                self._imu_data.angular_velocity.x,
                self._imu_data.angular_velocity.y,
                self._imu_data.angular_velocity.z,
            ],
            dtype=np.float32,
        )

        gravity = (
            quat2mat(
                [
                    self._imu_data.orientation.w,
                    self._imu_data.orientation.x,
                    self._imu_data.orientation.y,
                    self._imu_data.orientation.z,
                ]
            )
            @ euler2mat(0.0, 0.0, 0)
        ).T @ np.array([0, 0, -1], dtype=np.float32)

        joint_angles = (
            np.array(
                [
                    self._joint_state.position[self._joint_state.name.index(name)]
                    for name in ORDERED_RELEVANT_JOINT_NAMES
                ],
                dtype=np.float32,
            )
            - WALKREADY_STATE
        )

        phase = np.array([np.cos(self._phase), np.sin(self._phase)], dtype=np.float32).flatten()

        point = np.array([ball_pose.point.x, ball_pose.point.y], dtype=np.float32)
        if self._last_point is not None:
            diff = (point - self._last_point) / CONTROL_DT
        else:
            diff = np.array([0.0, 0.0], dtype=np.float32)
        self._last_point = point

        delta_t = (self.get_clock().now() - Time.from_msg(ball_pose.header.stamp)).nanoseconds / 1e9

        offset = diff * delta_t

        self.get_logger().info(f"Ball position: {point}, velocity: {diff}, offset: {offset}, delta_t: {delta_t}")

        point_corrected = point + offset

        obs = np.hstack(
            [
                point_corrected[0],
                point_corrected[1],
                gyro,
                gravity,
                joint_angles,
                self._previous_action,  # Previous action
                phase,
                kick_dir,
                0.5
            ]
        ).astype(np.float32)

        # Run the ONNX model
        onnx_input = {"obs": obs.reshape(1, -1)}
        onnx_pred = self._onnx_session.run(["continuous_actions"], onnx_input)[0][0]
        self._previous_action = onnx_pred

        # Publish the joint commands
        joint_command = JointCommand()
        joint_command.header.stamp = self._joint_state.header.stamp
        joint_command.joint_names = ORDERED_RELEVANT_JOINT_NAMES
        joint_command.positions = onnx_pred * 0.5 + WALKREADY_STATE
        joint_command.velocities = [-1.0] * len(ORDERED_RELEVANT_JOINT_NAMES)
        joint_command.accelerations = [-1.0] * len(ORDERED_RELEVANT_JOINT_NAMES)
        joint_command.kp = [30.0] * len(ORDERED_RELEVANT_JOINT_NAMES)
        joint_command.kd = [1.0] * len(ORDERED_RELEVANT_JOINT_NAMES)

        self._joint_command_pub.publish(joint_command)

        phase_tp1 = self._phase + self._phase_dt
        self._phase = np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi
        self._global_phase_pub.publish(Float32(data=self._phase[0]))


def main():
    import rclpy

    rclpy.init()
    ex = EventsExecutor()
    node = KickNode()
    ex.add_node(node)
    ex.spin()
    node.destroy_node()
    rclpy.try_shutdown()
