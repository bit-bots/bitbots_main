"""AprilTag bundle detection and pose estimation.

Ported from the gripper tracking of ma_flo (``ma_flo.gripper_tracking``): tags
are detected with pupil_apriltags and every detected member tag of a bundle
contributes its four corners to one joint ``cv2.solvePnP``. A bundle therefore
still yields a pose when it is only partially visible, and additional visible
member tags make the estimate more accurate and less ambiguous.
"""

from collections import defaultdict
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation

from bitbots_apriltag_localization.tag_map import BundleSpec, TagMap, TagSpec

#: The four tag corners in the tag frame, in the order pupil_apriltags reports
#: them, as fractions of the tag size. The tag frame follows the printed tag:
#: looking at the tag face, x points right, y up and z towards the observer.
#: In that frame pupil_apriltags reports the corners of a canonically printed
#: tag in the order bottom-left, bottom-right, top-right, top-left (verified
#: against the AprilRobotics reference tag images, see test_detector.py).
CORNER_OFFSETS = np.array(
    [
        [-0.5, -0.5, 0.0],
        [0.5, -0.5, 0.0],
        [0.5, 0.5, 0.0],
        [-0.5, 0.5, 0.0],
    ]
)


@dataclass(frozen=True)
class BundleDetection:
    """The estimated pose of one (possibly partially visible) bundle."""

    bundle: BundleSpec
    #: 4x4 pose of the bundle in the camera (optical) frame.
    camera_t_bundle: np.ndarray
    #: Ids of the member tags the pose was estimated from.
    tag_ids: tuple[int, ...]
    #: RMS reprojection error of all used corners in pixels.
    reprojection_error: float

    @property
    def num_corners(self) -> int:
        return 4 * len(self.tag_ids)


def tag_corners_in_bundle(tag: TagSpec) -> np.ndarray:
    """The tag's four corners expressed in the bundle frame, shape (4, 3)."""
    corners_tag = CORNER_OFFSETS * tag.size
    return corners_tag @ tag.bundle_t_tag[:3, :3].T + tag.bundle_t_tag[:3, 3]


class AprilTagBundleDetector:
    """Detects the bundles of a tag map in grayscale images."""

    def __init__(
        self,
        tag_map: TagMap,
        nthreads: int = 2,
        quad_decimate: float = 1.0,
        quad_sigma: float = 0.0,
        refine_edges: bool = True,
        decode_sharpening: float = 0.25,
    ):
        self._bundles_by_tag_id: dict[int, tuple[BundleSpec, TagSpec]] = {
            tag.id: (bundle, tag) for bundle in tag_map.bundles for tag in bundle.tags
        }
        self._detector = Detector(
            families=tag_map.tag_family,
            nthreads=nthreads,
            quad_decimate=quad_decimate,
            quad_sigma=quad_sigma,
            refine_edges=int(refine_edges),
            decode_sharpening=decode_sharpening,
        )
        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs: np.ndarray = np.zeros(5)

    def set_camera(self, camera_matrix: np.ndarray, dist_coeffs: Optional[np.ndarray] = None) -> None:
        """Sets the camera intrinsics (and optionally distortion coefficients)."""
        self._camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        self._dist_coeffs = np.zeros(5) if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64)

    @property
    def has_camera(self) -> bool:
        return self._camera_matrix is not None

    @property
    def camera_matrix(self) -> Optional[np.ndarray]:
        return self._camera_matrix

    @property
    def dist_coeffs(self) -> np.ndarray:
        return self._dist_coeffs

    def detect(self, gray_image: np.ndarray) -> list[BundleDetection]:
        """Detects all bundles of the map that are (partially) visible in the image."""
        assert self._camera_matrix is not None, "set_camera() must be called before detect()"

        # Group the detected tags of the image by the bundle they belong to.
        # Tags that are not part of the map are ignored.
        tags_by_bundle = defaultdict(list)
        for detection in self._detector.detect(gray_image):
            entry = self._bundles_by_tag_id.get(detection.tag_id)
            if entry is not None:
                bundle, tag = entry
                tags_by_bundle[bundle.name].append((detection, bundle, tag))

        detections = []
        for matched in tags_by_bundle.values():
            bundle = matched[0][1]

            # Joint PnP over the corners of all visible member tags: the 3D
            # corner positions in the bundle frame against their image points.
            object_points = np.concatenate([tag_corners_in_bundle(tag) for _, _, tag in matched]).astype(np.float32)
            image_points = np.concatenate([detection.corners for detection, _, _ in matched]).astype(np.float32)

            success, rvec, tvec = cv2.solvePnP(object_points, image_points, self._camera_matrix, self._dist_coeffs)
            if not success:
                continue

            camera_t_bundle = np.eye(4)
            camera_t_bundle[:3, :3], _ = cv2.Rodrigues(rvec)
            camera_t_bundle[:3, 3] = tvec.flatten()

            projected, _ = cv2.projectPoints(object_points, rvec, tvec, self._camera_matrix, self._dist_coeffs)
            reprojection_error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - image_points) ** 2, axis=1))))

            detections.append(
                BundleDetection(
                    bundle=bundle,
                    camera_t_bundle=camera_t_bundle,
                    tag_ids=tuple(detection.tag_id for detection, _, _ in matched),
                    reprojection_error=reprojection_error,
                )
            )

        return detections


def draw_detections(
    image: np.ndarray,
    detections: list[BundleDetection],
    detector: "AprilTagBundleDetector",
    max_reprojection_error: float,
) -> None:
    """Draws the reprojected bundle detections into a BGR debug image.

    Detections that pass the reprojection error threshold are drawn in green,
    dropped ones in red.
    """
    camera_matrix = detector.camera_matrix
    assert camera_matrix is not None, "the detector has no camera intrinsics yet"
    for detection in detections:
        color = (0, 255, 0) if detection.reprojection_error <= max_reprojection_error else (0, 0, 255)
        rvec, _ = cv2.Rodrigues(detection.camera_t_bundle[:3, :3])
        tvec = detection.camera_t_bundle[:3, 3]
        label_position = None
        for tag in detection.bundle.tags:
            if tag.id not in detection.tag_ids:
                continue
            projected, _ = cv2.projectPoints(
                tag_corners_in_bundle(tag), rvec, tvec, camera_matrix, detector.dist_coeffs
            )
            corners = projected.reshape(-1, 2).astype(np.int32)
            cv2.polylines(image, [corners], isClosed=True, color=color, thickness=2)
            if label_position is None:
                label_position = corners.min(axis=0)
        if label_position is not None:
            cv2.putText(
                image,
                f"{detection.bundle.name} ({detection.reprojection_error:.1f}px)",
                (int(label_position[0]), int(label_position[1]) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
            )


def camera_pose_from_detections(detections: list[BundleDetection]) -> np.ndarray:
    """Fuses bundle detections into one camera pose in the map frame.

    Every detection gives a full camera pose via the known map pose of its
    bundle. The poses are averaged, weighted by the number of corners the
    bundle pose was estimated from.
    """
    assert detections, "at least one detection is needed"

    camera_poses = [
        detection.bundle.map_t_bundle @ np.linalg.inv(detection.camera_t_bundle) for detection in detections
    ]
    weights = np.array([detection.num_corners for detection in detections], dtype=np.float64)

    map_t_camera = np.eye(4)
    map_t_camera[:3, 3] = np.average([pose[:3, 3] for pose in camera_poses], axis=0, weights=weights)
    map_t_camera[:3, :3] = (
        Rotation.from_matrix([pose[:3, :3] for pose in camera_poses]).mean(weights=weights).as_matrix()
    )
    return map_t_camera
