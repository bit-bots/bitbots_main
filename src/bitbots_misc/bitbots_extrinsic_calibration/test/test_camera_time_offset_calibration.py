import numpy as np
import pinocchio as pin
import pytest
from camera_time_offset_calibration import HeadImuPredictor, estimate_time_offset

# Minimal robot with a body IMU frame offset from the root and a two joint head with an IMU frame
URDF = """
<robot name="test">
  <link name="base_link"/>
  <link name="imu_link"/>
  <joint name="imu_joint" type="fixed">
    <origin xyz="0.01 -0.02 0.1" rpy="0.1 -0.2 0.3"/>
    <parent link="base_link"/>
    <child link="imu_link"/>
  </joint>
  <link name="head_yaw_link">
    <inertial><mass value="0.1"/><inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/></inertial>
  </link>
  <joint name="head_yaw_joint" type="revolute">
    <origin xyz="0.0 0.0 0.2"/>
    <parent link="base_link"/>
    <child link="head_yaw_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2" upper="2" effort="1" velocity="10"/>
  </joint>
  <link name="head_pitch_link">
    <inertial><mass value="0.1"/><inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/></inertial>
  </link>
  <joint name="head_pitch_joint" type="revolute">
    <origin xyz="0.0 0.0 0.03"/>
    <parent link="head_yaw_link"/>
    <child link="head_pitch_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="1" velocity="10"/>
  </joint>
  <link name="camera_link"/>
  <joint name="camera_joint" type="fixed">
    <origin xyz="0.05 0.005 0.06" rpy="0.0 0.1 0.0"/>
    <parent link="head_pitch_link"/>
    <child link="camera_link"/>
  </joint>
</robot>
"""

JOINT_NAMES = ["head_pitch_joint", "head_yaw_joint"]
GRAVITY = np.array([0.0, 0.0, -9.81])


def joint_positions(t: float) -> np.ndarray:
    return np.array([0.4 * np.sin(3.1 * t) + 0.2 * np.sin(7.3 * t), 0.8 * np.sin(1.7 * t + 0.5)])


def base_pose(t: float) -> pin.SE3:
    """Arbitrary smooth motion of the base in the world."""
    rotation = pin.rpy.rpyToMatrix(0.2 * np.sin(2.3 * t), 0.15 * np.sin(1.9 * t + 1.0), 0.5 * np.sin(0.7 * t))
    translation = np.array([0.3 * np.sin(1.1 * t), 0.1 * np.sin(2.9 * t), 0.4 + 0.05 * np.sin(4.1 * t)])
    return pin.SE3(rotation, translation)


class GroundTruth:
    """Simulates ideal IMUs by numerically differentiating the world poses of the IMU frames."""

    def __init__(self, predictor: HeadImuPredictor, step: float = 1e-4):
        self.predictor = predictor
        self.step = step

    def world_pose(self, t: float, frame_id: int) -> pin.SE3:
        q = self.predictor.joint_vectors(JOINT_NAMES, joint_positions(t)[None], neutral=True)[0]
        pin.framesForwardKinematics(self.predictor.model, self.predictor.data, q)
        return base_pose(t) * self.predictor.data.oMf[frame_id]

    def imu(self, t: float, frame_id: int) -> tuple[np.ndarray, np.ndarray]:
        before, now, after = (self.world_pose(t + dt, frame_id) for dt in (-self.step, 0.0, self.step))
        angular_velocity = pin.log3(before.rotation.T @ after.rotation) / (2 * self.step)
        acceleration = (after.translation - 2 * now.translation + before.translation) / self.step**2
        return angular_velocity, now.rotation.T @ (acceleration - GRAVITY)

    def joint_derivatives(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        before, now, after = (joint_positions(t + dt) for dt in (-self.step, 0.0, self.step))
        return now, (after - before) / (2 * self.step), (after - 2 * now + before) / self.step**2


@pytest.fixture
def predictor() -> HeadImuPredictor:
    return HeadImuPredictor(URDF, "imu_link", "camera_link")


def predict(predictor: HeadImuPredictor, truth: GroundTruth, times: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    joints = [truth.joint_derivatives(t) for t in times]
    body = [truth.imu(t, predictor.body_frame_id) for t in times]
    body_angular_velocity = np.array([b[0] for b in body])
    step = 1e-4
    body_angular_acceleration = np.array(
        [
            (truth.imu(t + step, predictor.body_frame_id)[0] - truth.imu(t - step, predictor.body_frame_id)[0])
            / (2 * step)
            for t in times
        ]
    )

    def vectors(index: int, neutral: bool) -> np.ndarray:
        return predictor.joint_vectors(JOINT_NAMES, np.array([j[index] for j in joints]), neutral=neutral)

    return predictor.predict(
        vectors(0, True),
        vectors(1, False),
        vectors(2, False),
        body_angular_velocity,
        body_angular_acceleration,
        np.array([b[1] for b in body]),
    )


def test_prediction_matches_ground_truth(predictor: HeadImuPredictor):
    truth = GroundTruth(predictor)
    times = np.linspace(0.0, 3.0, 25)
    angular_velocity, specific_force = predict(predictor, truth, times)

    expected = [truth.imu(t, predictor.head_frame_id) for t in times]
    np.testing.assert_allclose(angular_velocity, np.array([e[0] for e in expected]), atol=1e-5)
    np.testing.assert_allclose(specific_force, np.array([e[1] for e in expected]), atol=1e-3)


def test_unknown_joints_are_ignored(predictor: HeadImuPredictor):
    q = predictor.joint_vectors(["head_yaw_joint", "unknown_joint"], np.array([[0.5, 1.0]]), neutral=True)
    expected = pin.neutral(predictor.model)
    expected[predictor.joint_indices["head_yaw_joint"][0]] = 0.5
    np.testing.assert_array_equal(q[0], expected)


def test_missing_frame_raises():
    with pytest.raises(ValueError):
        HeadImuPredictor(URDF, "imu_link", "missing_link")


@pytest.mark.parametrize("offset", [-0.0423, 0.0, 0.031, 0.087])
def test_estimate_recovers_offset(predictor: HeadImuPredictor, offset: float):
    truth = GroundTruth(predictor)
    sample_rate = 200.0
    max_offset = 0.15
    times = np.arange(0.0, 4.0, 1.0 / sample_rate)
    angular_velocity, specific_force = predict(predictor, truth, times)
    predicted = np.hstack([angular_velocity, specific_force])

    # The measured head IMU is stamped later than the true time by the offset
    measured_true_times = np.arange(-0.5, 4.5, 1.0 / sample_rate)
    measured = np.array([np.hstack(truth.imu(t, predictor.head_frame_id)) for t in measured_true_times])
    measured += np.random.default_rng(0).normal(scale=0.01, size=measured.shape)

    estimate = estimate_time_offset(
        times, predicted, measured_true_times + offset, measured, max_offset, 1.0 / sample_rate, 1e-3
    )
    assert estimate is not None
    assert estimate.offset == pytest.approx(offset, abs=0.5 / sample_rate)
    assert estimate.correlation > 0.95


def test_estimate_without_motion_returns_none():
    times = np.linspace(0.0, 1.0, 100)
    constant = np.ones((times.size, 3))
    assert estimate_time_offset(times, constant, times, constant, 0.1, 0.01, 1e-3) is None
