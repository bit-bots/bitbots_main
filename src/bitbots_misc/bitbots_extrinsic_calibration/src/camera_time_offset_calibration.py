#!/usr/bin/env python3
"""Estimates the time offset of the head IMU (and therefore the camera) relative to the joint states and body IMU.

The head IMU signals are predicted from the body IMU and the joint states using the forward kinematics of the robot:
The body IMU measures the angular velocity and specific force of the base. The motion of the head relative to the base
is computed with the frame Jacobian (velocity) and its time variation (acceleration) at the head IMU frame. Combining
both by rigid body kinematics yields the expected angular velocity and linear acceleration at the head IMU.

The predicted signals are cross-correlated with the measured head IMU signals over a sliding window. The lag with the
highest correlation is the estimated offset. A positive offset means that the head IMU stamps are later than the true
measurement time, which matches the sign of the `camera_time_offset` parameter of the MuJoCo simulation.
"""

from collections import deque
from dataclasses import dataclass

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from scipy.signal import savgol_filter
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64, String

LWA = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED


@dataclass
class OffsetEstimate:
    offset: float
    correlation: float
    lags: np.ndarray
    correlations: np.ndarray


class HeadImuPredictor:
    """Predicts the head IMU signals from the body IMU and the joint states using a fixed base kinematic model."""

    def __init__(self, urdf: str, body_imu_frame: str, head_imu_frame: str):
        self.model = pin.buildModelFromXML(urdf)
        self.data = self.model.createData()
        for frame in (body_imu_frame, head_imu_frame):
            if not self.model.existFrame(frame):
                raise ValueError(f"Frame '{frame}' does not exist in the robot model")
        self.body_frame_id = self.model.getFrameId(body_imu_frame)
        self.head_frame_id = self.model.getFrameId(head_imu_frame)

        # Maps joint names to their index in the configuration and velocity vectors.
        # Only single degree of freedom joints can be filled from joint states, all others stay at their neutral value.
        self.joint_indices: dict[str, tuple[int, int]] = {
            name: (joint.idx_q, joint.idx_v)
            for name, joint in zip(self.model.names[1:], self.model.joints[1:], strict=True)
            if joint.nq == 1 and joint.nv == 1
        }

    def joint_vectors(self, names: list[str], values: np.ndarray, neutral: bool) -> np.ndarray:
        """Converts joint values (samples x joints) in the order of `names` to pinocchio vectors (samples x nq/nv)."""
        size = self.model.nq if neutral else self.model.nv
        result = np.tile(pin.neutral(self.model) if neutral else np.zeros(size), (values.shape[0], 1))
        for column, name in enumerate(names):
            if name in self.joint_indices:
                result[:, self.joint_indices[name][0 if neutral else 1]] = values[:, column]
        return result

    def predict(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        ddq: np.ndarray,
        body_angular_velocity: np.ndarray,
        body_angular_acceleration: np.ndarray,
        body_specific_force: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Predicts the angular velocity and specific force at the head IMU, expressed in the head IMU frame.

        All inputs are arrays with one row per sample. The body IMU quantities are expressed in the body IMU frame.
        The model root is treated as fixed, so all relative quantities are expressed in the root frame and the motion
        of the base is added afterwards from the body IMU.
        """
        angular_velocity = np.empty((q.shape[0], 3))
        specific_force = np.empty((q.shape[0], 3))
        for i in range(q.shape[0]):
            pin.computeJointJacobians(self.model, self.data, q[i])
            pin.computeJointJacobiansTimeVariation(self.model, self.data, q[i], dq[i])
            pin.updateFramePlacements(self.model, self.data)

            jacobian = pin.getFrameJacobian(self.model, self.data, self.head_frame_id, LWA)
            jacobian_dot = pin.getFrameJacobianTimeVariation(self.model, self.data, self.head_frame_id, LWA)
            # Motion of the head relative to the base, caused by the joints
            relative_twist = jacobian @ dq[i]
            relative_acceleration = jacobian @ ddq[i] + jacobian_dot @ dq[i]

            body_pose = self.data.oMf[self.body_frame_id]
            head_pose = self.data.oMf[self.head_frame_id]
            lever_arm = head_pose.translation - body_pose.translation

            # Motion of the base measured by the body IMU, rotated into the root frame
            omega = body_pose.rotation @ body_angular_velocity[i]
            alpha = body_pose.rotation @ body_angular_acceleration[i]
            force = body_pose.rotation @ body_specific_force[i]

            head_angular_velocity = omega + relative_twist[3:]
            # Transport of the specific force to the head, including the Euler, centripetal and Coriolis terms
            head_specific_force = (
                force
                + np.cross(alpha, lever_arm)
                + np.cross(omega, np.cross(omega, lever_arm))
                + 2 * np.cross(omega, relative_twist[:3])
                + relative_acceleration[:3]
            )

            angular_velocity[i] = head_pose.rotation.T @ head_angular_velocity
            specific_force[i] = head_pose.rotation.T @ head_specific_force
        return angular_velocity, specific_force


def smoothing_kernel_size(smoothing_window: float, sample_rate: float) -> int:
    """Returns the odd number of samples of a Savitzky-Golay window covering at least `smoothing_window` seconds."""
    size = max(5, int(np.ceil(smoothing_window * sample_rate)))
    return size if size % 2 == 1 else size + 1


def smooth(values: np.ndarray, kernel_size: int, sample_rate: float, deriv: int = 0) -> np.ndarray:
    """Smooths (and optionally differentiates) uniformly sampled values along the first axis."""
    return savgol_filter(values, kernel_size, polyorder=3, deriv=deriv, delta=1.0 / sample_rate, axis=0)


def estimate_time_offset(
    times: np.ndarray,
    predicted: np.ndarray,
    measured_times: np.ndarray,
    measured: np.ndarray,
    max_offset: float,
    lag_step: float,
    min_signal_std: float,
) -> OffsetEstimate | None:
    """Finds the lag that maximizes the cross-correlation between the predicted and the measured signals.

    The measured signals are evaluated at `times + lag` for each lag in [-max_offset, max_offset], so the measured
    stamps must cover that range. Channels whose predicted signal barely changes are ignored, as they carry no timing
    information. The correlation is the mean of the Pearson correlation of the remaining channels.
    The peak is refined to sub-step resolution with a parabolic fit.
    """
    channels = np.flatnonzero(predicted.std(axis=0) > min_signal_std)
    if channels.size == 0:
        return None
    predicted = predicted[:, channels]
    predicted = (predicted - predicted.mean(axis=0)) / predicted.std(axis=0)

    lags = np.arange(-max_offset, max_offset + lag_step / 2, lag_step)
    correlations = np.empty(lags.size)
    for i, lag in enumerate(lags):
        shifted = np.column_stack(
            [np.interp(times + lag, measured_times, measured[:, channel]) for channel in channels]
        )
        std = shifted.std(axis=0)
        std[std == 0] = np.inf
        correlations[i] = np.mean(np.mean(predicted * (shifted - shifted.mean(axis=0)), axis=0) / std)

    best = int(np.argmax(correlations))
    offset = lags[best]
    if 0 < best < lags.size - 1:
        left, center, right = correlations[best - 1 : best + 2]
        curvature = left - 2 * center + right
        if curvature < 0:
            offset += 0.5 * (left - right) / curvature * lag_step
    return OffsetEstimate(float(offset), float(correlations[best]), lags, correlations)


class TimeBuffer:
    """Stores stamped samples and drops the ones that are older than the configured duration."""

    def __init__(self):
        self.times: deque[float] = deque()
        self.values: deque[np.ndarray] = deque()

    def append(self, stamp: float, value: np.ndarray) -> None:
        if self.times and stamp <= self.times[-1]:
            return
        self.times.append(stamp)
        self.values.append(value)

    def prune(self, duration: float) -> None:
        while self.times and self.times[0] < self.times[-1] - duration:
            self.times.popleft()
            self.values.popleft()

    def arrays(self) -> tuple[np.ndarray, np.ndarray]:
        return np.array(self.times), np.array(self.values)

    def __len__(self) -> int:
        return len(self.times)


def stamp_to_sec(stamp) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


class CameraTimeOffsetCalibration(Node):
    def __init__(self):
        super().__init__("camera_time_offset_calibration")
        # All parameters except the frames are read on every estimation, so they can be changed at runtime
        self.declare_parameter("body_imu_frame", "imu_frame_uncalibrated")
        self.declare_parameter("head_imu_frame", "camera_link")
        self.declare_parameter("window_duration", 10.0)
        self.declare_parameter("max_offset", 0.2)
        self.declare_parameter("sample_rate", 200.0)
        self.declare_parameter("smoothing_window", 0.05)
        self.declare_parameter("estimation_period", 2.0)
        self.declare_parameter("min_signal_std", 0.05)
        self.declare_parameter("use_angular_velocity", False)
        self.declare_parameter("use_linear_acceleration", True)

        self.predictor: HeadImuPredictor | None = None
        self.joint_names: list[str] | None = None
        self.joint_buffer = TimeBuffer()
        self.body_imu_buffer = TimeBuffer()
        self.head_imu_buffer = TimeBuffer()

        self.create_subscription(
            String,
            "robot_description",
            self.robot_description_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.create_subscription(Imu, "imu/data", self.body_imu_callback, 10)
        self.create_subscription(Imu, "zed/zed_node/imu/data", self.head_imu_callback, 10)
        self.offset_publisher = self.create_publisher(Float64, "camera_time_offset_estimate", 1)

        self.estimation_timer = self.create_timer(self.get_parameter("estimation_period").value, self.estimate)

    def robot_description_callback(self, msg: String) -> None:
        try:
            self.predictor = HeadImuPredictor(
                msg.data, self.get_parameter("body_imu_frame").value, self.get_parameter("head_imu_frame").value
            )
        except ValueError as error:
            self.get_logger().error(str(error))

    def buffer_duration(self) -> float:
        return self.get_parameter("window_duration").value + 2 * self.get_parameter("max_offset").value + 1.0

    def joint_state_callback(self, msg: JointState) -> None:
        if self.joint_names is None:
            self.joint_names = list(msg.name)
        if list(msg.name) != self.joint_names:
            self.get_logger().warn("Joint state names changed, dropping buffered joint states")
            self.joint_names = list(msg.name)
            self.joint_buffer = TimeBuffer()
        self.joint_buffer.append(stamp_to_sec(msg.header.stamp), np.array(msg.position))
        self.joint_buffer.prune(self.buffer_duration())

    def body_imu_callback(self, msg: Imu) -> None:
        self.append_imu(self.body_imu_buffer, msg)

    def head_imu_callback(self, msg: Imu) -> None:
        self.append_imu(self.head_imu_buffer, msg)

    def append_imu(self, buffer: TimeBuffer, msg: Imu) -> None:
        w, a = msg.angular_velocity, msg.linear_acceleration
        buffer.append(stamp_to_sec(msg.header.stamp), np.array([w.x, w.y, w.z, a.x, a.y, a.z]))
        buffer.prune(self.buffer_duration())

    def estimate(self) -> None:
        # Keep the timer in sync with the parameter
        period_ns = int(self.get_parameter("estimation_period").value * 1e9)
        if self.estimation_timer.timer_period_ns != period_ns:
            self.estimation_timer.timer_period_ns = period_ns

        if self.predictor is None or self.joint_names is None:
            self.get_logger().info("Waiting for robot description and joint states", throttle_duration_sec=10.0)
            return
        if min(len(self.joint_buffer), len(self.body_imu_buffer), len(self.head_imu_buffer)) < 2:
            self.get_logger().info("Waiting for joint states and IMU data", throttle_duration_sec=10.0)
            return

        window_duration = self.get_parameter("window_duration").value
        max_offset = self.get_parameter("max_offset").value
        sample_rate = self.get_parameter("sample_rate").value
        kernel_size = smoothing_kernel_size(self.get_parameter("smoothing_window").value, sample_rate)

        joint_times, joint_positions = self.joint_buffer.arrays()
        body_times, body_imu = self.body_imu_buffer.arrays()
        head_times, head_imu = self.head_imu_buffer.arrays()

        # Common grid for which the measured head IMU signal is available for all lags
        end = min(joint_times[-1], body_times[-1], head_times[-1] - max_offset)
        start = max(joint_times[0], body_times[0], head_times[0] + max_offset, end - window_duration)
        times = np.arange(start, end, 1.0 / sample_rate)
        if times.size < 2 * kernel_size:
            self.get_logger().info("Not enough overlapping data in the window yet", throttle_duration_sec=10.0)
            return

        def resample(sample_times: np.ndarray, values: np.ndarray, at: np.ndarray) -> np.ndarray:
            return np.column_stack([np.interp(at, sample_times, values[:, i]) for i in range(values.shape[1])])

        positions = resample(joint_times, joint_positions, times)
        q = self.predictor.joint_vectors(self.joint_names, smooth(positions, kernel_size, sample_rate), neutral=True)
        dq = self.predictor.joint_vectors(
            self.joint_names, smooth(positions, kernel_size, sample_rate, deriv=1), neutral=False
        )
        ddq = self.predictor.joint_vectors(
            self.joint_names, smooth(positions, kernel_size, sample_rate, deriv=2), neutral=False
        )

        body = resample(body_times, body_imu, times)
        predicted_angular_velocity, predicted_specific_force = self.predictor.predict(
            q,
            dq,
            ddq,
            smooth(body[:, :3], kernel_size, sample_rate),
            smooth(body[:, :3], kernel_size, sample_rate, deriv=1),
            smooth(body[:, 3:], kernel_size, sample_rate),
        )

        # Smooth the measured signal on its own uniform grid, so it is filtered the same way as the prediction
        head_grid = np.arange(head_times[0], head_times[-1], 1.0 / sample_rate)
        measured = smooth(resample(head_times, head_imu, head_grid), kernel_size, sample_rate)

        use_angular_velocity = self.get_parameter("use_angular_velocity").value
        use_linear_acceleration = self.get_parameter("use_linear_acceleration").value
        channels = [i for i in range(3) if use_angular_velocity] + [i for i in range(3, 6) if use_linear_acceleration]
        if not channels:
            self.get_logger().warn("Both angular velocity and linear acceleration are disabled")
            return
        predicted = np.hstack([predicted_angular_velocity, predicted_specific_force])[:, channels]

        estimate = estimate_time_offset(
            times,
            predicted,
            head_grid,
            measured[:, channels],
            max_offset,
            1.0 / sample_rate,
            self.get_parameter("min_signal_std").value,
        )
        if estimate is None:
            self.get_logger().warn("Not enough head motion in the window to estimate the offset")
            return

        self.get_logger().info(
            f"Estimated head IMU time offset: {estimate.offset * 1000:.1f} ms (correlation {estimate.correlation:.3f})"
        )
        self.offset_publisher.publish(Float64(data=estimate.offset))


def main(args=None):
    rclpy.init(args=args)
    node = CameraTimeOffsetCalibration()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
