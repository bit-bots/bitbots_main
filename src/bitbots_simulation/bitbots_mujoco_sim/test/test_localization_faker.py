import numpy as np
import pytest
from bitbots_mujoco_sim.field_publisher import field_map_msg
from bitbots_mujoco_sim.localization_faker import compute_map_to_odom, planar_transform
from geometry_msgs.msg import Pose
from transforms3d.euler import euler2quat


def make_pose(x: float, y: float, yaw: float, z: float = 0.0, roll: float = 0.0, pitch: float = 0.0) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    q = euler2quat(roll, pitch, yaw, axes="sxyz")
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = q
    return pose


def map_to_base(map_to_odom: tuple[float, float, float, float], odom_pose: Pose) -> np.ndarray:
    x, y, _, yaw = map_to_odom
    odom_yaw = 2 * np.arctan2(odom_pose.orientation.z, odom_pose.orientation.w)
    return planar_transform(x, y, yaw) @ planar_transform(odom_pose.position.x, odom_pose.position.y, odom_yaw)


def test_identity_odom_reproduces_ground_truth():
    world_pose = make_pose(1.0, -2.0, 0.5)
    x, y, z, yaw = compute_map_to_odom(world_pose, make_pose(0.0, 0.0, 0.0), flip_side=False)
    assert (x, y, z, yaw) == pytest.approx((1.0, -2.0, 0.0, 0.5))


@pytest.mark.parametrize("flip_side", [False, True])
def test_map_to_base_matches_ground_truth(flip_side):
    world_pose = make_pose(1.0, -3.0, 1.2)
    odom_pose = make_pose(0.4, 0.3, -0.7)
    result = map_to_base(compute_map_to_odom(world_pose, odom_pose, flip_side), odom_pose)

    expected = planar_transform(1.0, -3.0, 1.2)
    if flip_side:
        expected = planar_transform(0.0, 0.0, np.pi) @ expected
    np.testing.assert_allclose(result, expected, atol=1e-9)


def test_flip_side_mirrors_position_through_center():
    world_pose = make_pose(2.0, 1.0, 0.0)
    x, y, z, yaw = compute_map_to_odom(world_pose, make_pose(0.0, 0.0, 0.0), flip_side=True)
    assert (x, y, z, abs(yaw)) == pytest.approx((-2.0, -1.0, 0.0, np.pi))


def test_roll_pitch_and_height_are_ignored():
    upright = compute_map_to_odom(make_pose(1.0, 2.0, 0.3), make_pose(0.0, 0.0, 0.0), flip_side=False)
    tilted = compute_map_to_odom(
        make_pose(1.0, 2.0, 0.3, z=0.4, roll=0.1, pitch=-0.15), make_pose(0.0, 0.0, 0.0), flip_side=False
    )
    assert tilted[:2] == pytest.approx(upright[:2], abs=1e-9)
    assert tilted[2] == pytest.approx(0.4)
    assert tilted[3] == pytest.approx(upright[3], abs=1e-9)


def test_nonzero_odom_height_is_inverted():
    world_pose = make_pose(1.0, -2.0, 0.5, z=0.25)
    flat = compute_map_to_odom(world_pose, make_pose(0.4, 0.3, -0.7), flip_side=False)
    elevated = compute_map_to_odom(world_pose, make_pose(0.4, 0.3, -0.7, z=0.6), flip_side=False)

    assert elevated[:2] == pytest.approx(flat[:2])
    assert elevated[2] == pytest.approx(-0.35)
    assert elevated[3] == pytest.approx(flat[3])


def test_field_map_msg_is_centered_and_inverted():
    field_map = np.array([[0, 100, 50], [100, 0, 25]], dtype=np.uint8)
    msg = field_map_msg(field_map, "map")
    assert (msg.info.width, msg.info.height) == (3, 2)
    assert msg.info.origin.position.x == pytest.approx(-msg.info.width / 2 * msg.info.resolution)
    assert msg.info.origin.position.y == pytest.approx(-msg.info.height / 2 * msg.info.resolution)
    assert list(msg.data) == [100, 0, 50, 0, 100, 75]
