"""Tests the detector against synthetically rendered tag images.

Tags are rendered with cv2.aruco into an artificial camera image at known
poses, detected with pupil_apriltags and the recovered poses are compared to
the ground truth. This validates the whole chain: the tag frame convention,
the corner order, the joint bundle PnP and the map-frame fusion.
"""

import cv2
import numpy as np
from bitbots_apriltag_localization.detector import (
    AprilTagBundleDetector,
    camera_pose_from_detections,
)
from bitbots_apriltag_localization.tag_map import BundleSpec, TagMap, TagSpec, pose_to_matrix
from scipy.spatial.transform import Rotation

CAMERA_MATRIX = np.array([[800.0, 0.0, 640.0], [0.0, 800.0, 360.0], [0.0, 0.0, 1.0]])
RESOLUTION = (1280, 720)

#: A tag 1 m in front of the camera, facing it: the tag's x (right) stays the
#: camera's x, its y (up) is the camera's -y (image y points down) and its z
#: (out of the tag face) points back at the camera, i.e. the camera's -z.
FACING_CAMERA = pose_to_matrix({"position": [0.0, 0.0, 1.0], "rpy_deg": [180.0, 0.0, 0.0]}, "test")


def _pose(position, rpy_deg=(0.0, 0.0, 0.0)) -> np.ndarray:
    return pose_to_matrix({"position": list(position), "rpy_deg": list(rpy_deg)}, "test")


def _render_tag(canvas: np.ndarray, camera_t_tag: np.ndarray, tag_id: int, size: float) -> None:
    """Draws a tag at the given camera-frame pose into the (grayscale) canvas."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    # cv2.aruco renders the AprilTag families rotated by 180 degrees compared
    # to the canonical AprilRobotics tag images, so rotate the marker back.
    marker = cv2.rotate(cv2.aruco.generateImageMarker(dictionary, tag_id, 160), cv2.ROTATE_180)
    # The corners of the black tag square in the tag frame, in the pixel order
    # of the marker bitmap (top-left, top-right, bottom-right, bottom-left).
    corners_tag = np.array([[-0.5, 0.5, 0.0], [0.5, 0.5, 0.0], [0.5, -0.5, 0.0], [-0.5, -0.5, 0.0]]) * size
    corners_camera = corners_tag @ camera_t_tag[:3, :3].T + camera_t_tag[:3, 3]
    assert (corners_camera[:, 2] > 0).all(), "tag must be in front of the camera"
    projected = (corners_camera / corners_camera[:, 2:]) @ CAMERA_MATRIX.T
    source = np.array([[0, 0], [160, 0], [160, 160], [0, 160]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(source, projected[:, :2].astype(np.float32))
    warped = cv2.warpPerspective(marker, homography, canvas.shape[::-1])
    mask = cv2.warpPerspective(np.full_like(marker, 255), homography, canvas.shape[::-1])
    canvas[mask > 127] = warped[mask > 127]


def _render_map(tag_map: TagMap, map_t_camera: np.ndarray) -> np.ndarray:
    """Renders all tags of a map as seen by a camera at the given map pose."""
    canvas = np.full(RESOLUTION[::-1], 255, dtype=np.uint8)
    camera_t_map = np.linalg.inv(map_t_camera)
    for bundle in tag_map.bundles:
        for tag in bundle.tags:
            _render_tag(canvas, camera_t_map @ bundle.map_t_bundle @ tag.bundle_t_tag, tag.id, tag.size)
    return canvas


def _detector_for(tag_map: TagMap) -> AprilTagBundleDetector:
    detector = AprilTagBundleDetector(tag_map)
    detector.set_camera(CAMERA_MATRIX)
    return detector


def _assert_pose_close(actual: np.ndarray, expected: np.ndarray, position_tol=0.01, angle_tol_deg=1.0):
    position_error = np.linalg.norm(actual[:3, 3] - expected[:3, 3])
    angle_error = np.degrees(Rotation.from_matrix(actual[:3, :3].T @ expected[:3, :3]).magnitude())
    assert position_error < position_tol, f"position off by {position_error:.4f} m"
    assert angle_error < angle_tol_deg, f"rotation off by {angle_error:.2f} deg"


def _single_tag_map(tag_id: int = 0, size: float = 0.16) -> TagMap:
    tag = TagSpec(id=tag_id, size=size, bundle_t_tag=np.eye(4))
    bundle = BundleSpec(name=f"tag_{tag_id}", map_t_bundle=np.eye(4), tags=(tag,))
    return TagMap(tag_family="tag36h11", bundles=(bundle,))


def test_single_tag_pose():
    tag_map = _single_tag_map()
    # A slightly rotated and shifted tag so that no axis is degenerate.
    camera_t_tag = FACING_CAMERA @ _pose((0.05, -0.03, 0.0), (10.0, -15.0, 5.0))

    canvas = np.full(RESOLUTION[::-1], 255, dtype=np.uint8)
    _render_tag(canvas, camera_t_tag, tag_id=0, size=0.16)
    detections = _detector_for(tag_map).detect(canvas)

    assert len(detections) == 1
    assert detections[0].tag_ids == (0,)
    assert detections[0].reprojection_error < 1.0
    _assert_pose_close(detections[0].camera_t_bundle, camera_t_tag)


def test_bundle_joint_pose():
    bundle = BundleSpec(
        name="board",
        map_t_bundle=np.eye(4),
        tags=(
            TagSpec(id=10, size=0.1, bundle_t_tag=_pose((-0.15, 0.0, 0.0))),
            TagSpec(id=11, size=0.1, bundle_t_tag=_pose((0.15, 0.0, 0.0), (0.0, -30.0, 0.0))),
        ),
    )
    tag_map = TagMap(tag_family="tag36h11", bundles=(bundle,))
    camera_t_bundle = FACING_CAMERA @ _pose((0.0, 0.05, 0.0), (5.0, 10.0, -8.0))

    canvas = np.full(RESOLUTION[::-1], 255, dtype=np.uint8)
    for tag in bundle.tags:
        _render_tag(canvas, camera_t_bundle @ tag.bundle_t_tag, tag.id, tag.size)
    detections = _detector_for(tag_map).detect(canvas)

    assert len(detections) == 1
    assert sorted(detections[0].tag_ids) == [10, 11]
    assert detections[0].num_corners == 8
    _assert_pose_close(detections[0].camera_t_bundle, camera_t_bundle)


def test_partially_visible_bundle():
    bundle = BundleSpec(
        name="board",
        map_t_bundle=np.eye(4),
        tags=(
            TagSpec(id=10, size=0.16, bundle_t_tag=_pose((-0.15, 0.0, 0.0))),
            TagSpec(id=11, size=0.16, bundle_t_tag=_pose((0.15, 0.0, 0.0))),
        ),
    )
    tag_map = TagMap(tag_family="tag36h11", bundles=(bundle,))
    camera_t_bundle = FACING_CAMERA @ _pose((0.0, 0.0, 0.0), (10.0, -12.0, 4.0))

    # Only one of the two member tags is visible.
    canvas = np.full(RESOLUTION[::-1], 255, dtype=np.uint8)
    _render_tag(canvas, camera_t_bundle @ bundle.tags[0].bundle_t_tag, bundle.tags[0].id, bundle.tags[0].size)
    detections = _detector_for(tag_map).detect(canvas)

    assert len(detections) == 1
    assert detections[0].tag_ids == (10,)
    _assert_pose_close(detections[0].camera_t_bundle, camera_t_bundle)


def test_different_tag_sizes():
    tags = (
        TagSpec(id=0, size=0.2, bundle_t_tag=_pose((-0.2, 0.0, 0.0))),
        TagSpec(id=1, size=0.1, bundle_t_tag=_pose((0.2, 0.0, 0.0))),
    )
    bundle = BundleSpec(name="mixed", map_t_bundle=np.eye(4), tags=tags)
    tag_map = TagMap(tag_family="tag36h11", bundles=(bundle,))
    camera_t_bundle = FACING_CAMERA @ _pose((0.0, 0.0, 0.2), (8.0, 15.0, -3.0))

    canvas = np.full(RESOLUTION[::-1], 255, dtype=np.uint8)
    for tag in tags:
        _render_tag(canvas, camera_t_bundle @ tag.bundle_t_tag, tag.id, tag.size)
    detections = _detector_for(tag_map).detect(canvas)

    assert len(detections) == 1
    assert detections[0].reprojection_error < 1.0
    _assert_pose_close(detections[0].camera_t_bundle, camera_t_bundle)


def test_unknown_tags_are_ignored():
    tag_map = _single_tag_map(tag_id=0)
    canvas = np.full(RESOLUTION[::-1], 255, dtype=np.uint8)
    _render_tag(canvas, FACING_CAMERA, tag_id=42, size=0.16)
    assert _detector_for(tag_map).detect(canvas) == []


def test_camera_pose_from_detections():
    """End to end: several bundles on a wall localize the camera in the map frame."""
    tag_map = TagMap(
        tag_family="tag36h11",
        bundles=(
            # A standalone tag and a two-tag board on a wall at x = 3 m, both
            # facing -x (into the room).
            BundleSpec(
                name="tag_0",
                map_t_bundle=_pose((3.0, -0.3, 0.5), (90.0, 0.0, -90.0)),
                tags=(TagSpec(id=0, size=0.16, bundle_t_tag=np.eye(4)),),
            ),
            BundleSpec(
                name="board",
                map_t_bundle=_pose((3.0, 0.4, 0.5), (90.0, 0.0, -90.0)),
                tags=(
                    TagSpec(id=10, size=0.16, bundle_t_tag=_pose((-0.25, 0.0, 0.0))),
                    TagSpec(id=11, size=0.16, bundle_t_tag=_pose((0.25, 0.0, 0.0))),
                ),
            ),
        ),
    )

    # The camera stands in the room and looks along +x at the wall: camera z
    # (view direction) is the map's x, camera x (image right) the map's -y and
    # camera y (image down) the map's -z; plus a slight tilt so that nothing
    # is perfectly axis aligned.
    map_t_camera = _pose((1.5, 0.0, 0.5), (0.0, 90.0, 0.0)) @ _pose((0.0, 0.0, 0.0), (0.0, 0.0, -90.0))
    map_t_camera = map_t_camera @ _pose((0.0, 0.0, 0.0), (2.0, -3.0, 5.0))

    detections = _detector_for(tag_map).detect(_render_map(tag_map, map_t_camera))
    assert len(detections) == 2

    # The tags are only ~85 px across in the rendered image, so the corner
    # aliasing leaves a centimeter-level depth error at this distance. A wrong
    # frame convention would instead be off by decimeters and tens of degrees.
    _assert_pose_close(camera_pose_from_detections(detections), map_t_camera, position_tol=0.03, angle_tol_deg=1.5)
