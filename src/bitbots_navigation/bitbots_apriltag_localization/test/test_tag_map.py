from pathlib import Path

import numpy as np
import pytest
from bitbots_apriltag_localization.tag_map import load_tag_map


def _write(tmp_path: Path, content: str) -> Path:
    path = tmp_path / "tag_map.yaml"
    path.write_text(content)
    return path


def test_shipped_map_loads():
    tag_map = load_tag_map(Path(__file__).parent.parent / "config" / "tag_map.yaml")
    assert tag_map.tag_family == "tag36h11"

    # The 5x5 ground grid: one rigid bundle of ids 52-76 in an S pattern with
    # 2 m spacing.
    bundle = next(bundle for bundle in tag_map.bundles if bundle.name == "ground_grid")
    grid = {tag.id: bundle.map_t_bundle @ tag.bundle_t_tag for tag in bundle.tags}
    assert len(grid) == 25
    assert np.allclose(grid[52][:3, 3], [-4.0, -4.0, 0.0])
    assert np.allclose(grid[76][:3, 3], [4.0, 4.0, 0.0])
    for tag_id, pose in grid.items():
        # Flat on the ground, top edge of the tag pointing along +y.
        assert np.allclose(pose[:3, :3], np.eye(3)), f"tag {tag_id} is not flat on the ground"
        assert pose[2, 3] == 0.0
    for tag_id in range(52, 76):
        # The S pattern makes consecutive ids physical neighbors.
        step = np.linalg.norm(grid[tag_id + 1][:3, 3] - grid[tag_id][:3, 3])
        assert step == 2.0, f"tags {tag_id} and {tag_id + 1} are {step} m apart"


def test_standalone_tags_and_bundles(tmp_path):
    tag_map = load_tag_map(
        _write(
            tmp_path,
            """
            default_tag_size: 0.16
            tags:
              - id: 0
                pose:
                  position: [3.0, 0.0, 0.5]
                  rpy_deg: [90.0, 0.0, -90.0]
              - id: 1
                size: 0.2
                pose:
                  position: [1.0, 2.0, 0.5]
                  quat_xyzw: [0.0, 0.0, 0.0, 1.0]
            bundles:
              - name: board
                pose:
                  position: [0.0, 2.0, 0.5]
                  rpy_deg: [90.0, 0.0, 0.0]
                tags:
                  - id: 10
                    size: 0.1
                    pose: { position: [-0.25, 0.0, 0.0] }
                  - id: 11
                    size: 0.1
                    pose: { position: [0.25, 0.0, 0.0] }
            """,
        )
    )

    standalone, quat_tag, board = tag_map.bundles

    # A standalone tag becomes a single-tag bundle with the tag at its origin.
    assert standalone.name == "tag_0"
    assert standalone.tags[0].size == 0.16  # default size applied
    assert np.allclose(standalone.tags[0].bundle_t_tag, np.eye(4))
    # rpy [90, 0, -90] puts the tag face (its z axis) along the map's -x.
    assert np.allclose(standalone.map_t_bundle[:3, 2], [-1.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(standalone.map_t_bundle[:3, 3], [3.0, 0.0, 0.5])

    assert quat_tag.tags[0].size == 0.2
    assert np.allclose(quat_tag.map_t_bundle[:3, :3], np.eye(3))

    assert board.name == "board"
    assert [tag.id for tag in board.tags] == [10, 11]
    assert np.allclose(board.tags[0].bundle_t_tag[:3, 3], [-0.25, 0.0, 0.0])


def test_duplicate_tag_ids_rejected(tmp_path):
    with pytest.raises(ValueError, match="more than once"):
        load_tag_map(
            _write(
                tmp_path,
                """
                default_tag_size: 0.16
                tags:
                  - id: 0
                bundles:
                  - name: board
                    tags:
                      - id: 0
                """,
            )
        )


def test_missing_size_rejected(tmp_path):
    with pytest.raises(ValueError, match="no 'size'"):
        load_tag_map(_write(tmp_path, "tags:\n  - id: 0\n"))


def test_ambiguous_orientation_rejected(tmp_path):
    with pytest.raises(ValueError, match="not both"):
        load_tag_map(
            _write(
                tmp_path,
                """
                default_tag_size: 0.16
                tags:
                  - id: 0
                    pose:
                      rpy_deg: [0.0, 0.0, 0.0]
                      quat_xyzw: [0.0, 0.0, 0.0, 1.0]
                """,
            )
        )


def test_unknown_keys_rejected(tmp_path):
    with pytest.raises(ValueError, match="unknown keys"):
        load_tag_map(_write(tmp_path, "default_tag_size: 0.16\ntag_size: 0.1\ntags:\n  - id: 0\n"))


def test_empty_bundle_rejected(tmp_path):
    with pytest.raises(ValueError, match="no tags"):
        load_tag_map(_write(tmp_path, "bundles:\n  - name: board\n"))


def test_empty_map_rejected(tmp_path):
    with pytest.raises(ValueError, match="no tags at all"):
        load_tag_map(_write(tmp_path, "default_tag_size: 0.16\n"))
