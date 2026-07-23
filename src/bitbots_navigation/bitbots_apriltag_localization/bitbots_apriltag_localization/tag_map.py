"""Loading of the AprilTag map YAML.

The map file defines which AprilTags are placed where in the world. Standalone
tags and rigid multi-tag bundles are both represented as bundles internally: a
standalone tag is simply a bundle with a single member tag at the bundle
origin. This way the detector only ever has to deal with one concept.

Conventions of the map file (see ``config/tag_map.yaml`` for a full example):

* Poses are given as a ``position: [x, y, z]`` in meters and an orientation as
  either ``rpy_deg: [roll, pitch, yaw]`` in degrees (extrinsic, around the
  fixed x/y/z axes, like the ROS RPY convention) or ``quat_xyzw: [x, y, z, w]``.
* The tag frame follows the printed tag: looking at the tag face, x points to
  the right, y up and z out of the tag towards the observer.
* ``size`` is the edge length of the (outer) black square in meters, without
  the white margin around it.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import yaml  # type: ignore[import-untyped]
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class TagSpec:
    """A single AprilTag as a member of a bundle."""

    id: int
    #: Edge length of the black tag square in meters.
    size: float
    #: 4x4 pose of the tag in the bundle frame.
    bundle_t_tag: np.ndarray


@dataclass(frozen=True)
class BundleSpec:
    """A rigid arrangement of tags with a known pose in the map frame."""

    name: str
    #: 4x4 pose of the bundle in the map frame.
    map_t_bundle: np.ndarray
    tags: tuple[TagSpec, ...]


@dataclass(frozen=True)
class TagMap:
    tag_family: str
    bundles: tuple[BundleSpec, ...]


def pose_to_matrix(pose: dict, context: str) -> np.ndarray:
    """Builds a 4x4 homogeneous transform from a pose dict of the map file."""
    if not isinstance(pose, dict):
        raise ValueError(f"{context}: 'pose' must be a mapping, got {type(pose).__name__}")

    position = pose.get("position", [0.0, 0.0, 0.0])
    if len(position) != 3:
        raise ValueError(f"{context}: 'position' must have three entries")

    has_rpy = "rpy_deg" in pose
    has_quat = "quat_xyzw" in pose
    if has_rpy and has_quat:
        raise ValueError(f"{context}: give either 'rpy_deg' or 'quat_xyzw', not both")
    if has_rpy:
        rotation = Rotation.from_euler("xyz", pose["rpy_deg"], degrees=True)
    elif has_quat:
        rotation = Rotation.from_quat(pose["quat_xyzw"])
    else:
        rotation = Rotation.identity()

    unknown = set(pose) - {"position", "rpy_deg", "quat_xyzw"}
    if unknown:
        raise ValueError(f"{context}: unknown pose keys {sorted(unknown)}")

    matrix = np.eye(4)
    matrix[:3, :3] = rotation.as_matrix()
    matrix[:3, 3] = position
    return matrix


def _parse_tag(entry: dict, default_size: Optional[float], context: str) -> TagSpec:
    if "id" not in entry:
        raise ValueError(f"{context}: a tag needs an 'id'")
    tag_id = int(entry["id"])
    size = entry.get("size", default_size)
    if size is None:
        raise ValueError(f"{context}: tag {tag_id} has no 'size' and the map defines no 'default_tag_size'")
    pose = pose_to_matrix(entry.get("pose", {}), f"{context}: tag {tag_id}")
    return TagSpec(id=tag_id, size=float(size), bundle_t_tag=pose)


def load_tag_map(path: Path) -> TagMap:
    """Loads and validates a tag map YAML file."""
    with open(path) as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: the tag map must be a mapping")

    unknown = set(raw) - {"tag_family", "default_tag_size", "tags", "bundles"}
    if unknown:
        raise ValueError(f"{path}: unknown keys {sorted(unknown)}")

    tag_family = str(raw.get("tag_family", "tag36h11"))
    default_size = raw.get("default_tag_size")

    bundles: list[BundleSpec] = []

    # Standalone tags become single-tag bundles with the tag at the bundle origin.
    for entry in raw.get("tags") or []:
        tag = _parse_tag({**entry, "pose": {}}, default_size, str(path))
        map_t_tag = pose_to_matrix(entry.get("pose", {}), f"{path}: tag {tag.id}")
        bundles.append(BundleSpec(name=f"tag_{tag.id}", map_t_bundle=map_t_tag, tags=(tag,)))

    for entry in raw.get("bundles") or []:
        if "name" not in entry:
            raise ValueError(f"{path}: a bundle needs a 'name'")
        name = str(entry["name"])
        map_t_bundle = pose_to_matrix(entry.get("pose", {}), f"{path}: bundle {name}")
        member_tags = tuple(
            _parse_tag(tag_entry, default_size, f"{path}: bundle {name}") for tag_entry in entry.get("tags") or []
        )
        if not member_tags:
            raise ValueError(f"{path}: bundle {name} has no tags")
        bundles.append(BundleSpec(name=name, map_t_bundle=map_t_bundle, tags=member_tags))

    if not bundles:
        raise ValueError(f"{path}: the tag map defines no tags at all")

    names = [bundle.name for bundle in bundles]
    if len(names) != len(set(names)):
        raise ValueError(f"{path}: duplicate bundle names")

    tag_ids = [tag.id for bundle in bundles for tag in bundle.tags]
    duplicates = {tag_id for tag_id in tag_ids if tag_ids.count(tag_id) > 1}
    if duplicates:
        raise ValueError(f"{path}: tag ids {sorted(duplicates)} appear more than once in the map")

    return TagMap(tag_family=tag_family, bundles=tuple(bundles))
