import hashlib
import json
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import prepare_real_dataset as preparation
import pytest


def interval(first=3, last=5, bag="recording", reason="Visuell geprüfter Lichtausfall"):
    return {"bag": bag, "first_frame": first, "last_frame": last, "reason": reason}


def annotation(frame):
    return {
        "bag": "recording",
        "frame": frame,
        "boxes": [[1, 1, 3, 3]],
        "reviewed": True,
        "split": "train",
        "review_method": "manual",
    }


@pytest.fixture
def camera_bag(tmp_path, monkeypatch):
    bag = tmp_path / "bags" / "recording"
    bag.mkdir(parents=True)
    (bag / "recording.mcap").write_bytes(b"test-bag" + b"\x89MCAP0\r\n")

    class Reader:
        def __init__(self):
            self.frame = 0

        def open(self, storage, converter):
            pass

        def set_filter(self, topic_filter):
            assert topic_filter.topics == [preparation.IMAGE_TOPIC]

        def has_next(self):
            return self.frame < 8

        def read_next(self):
            self.frame += 1
            return preparation.IMAGE_TOPIC, self.frame, self.frame * 100

    class Bridge:
        def imgmsg_to_cv2(self, message, desired_encoding):
            assert desired_encoding == "bgr8"
            return np.full((10, 10, 3), message, dtype=np.uint8)

    serialization = SimpleNamespace(deserialize_message=lambda serialized, message_type: serialized)
    messages = SimpleNamespace(Image=object)
    modules = {
        "rosbag2_py": SimpleNamespace(
            SequentialReader=Reader,
            StorageOptions=lambda **kwargs: SimpleNamespace(**kwargs),
            ConverterOptions=lambda *args: None,
            StorageFilter=lambda **kwargs: SimpleNamespace(**kwargs),
        ),
        "cv_bridge": SimpleNamespace(CvBridge=Bridge),
        "rclpy": SimpleNamespace(serialization=serialization),
        "rclpy.serialization": serialization,
        "sensor_msgs": SimpleNamespace(msg=messages),
        "sensor_msgs.msg": messages,
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    return bag.parent


@pytest.mark.parametrize(
    "invalid",
    [
        {},
        "intervals",
        [None],
        [[]],
        [{}],
        [{"bag": "recording", "first_frame": 1, "last_frame": 2}],
        [dict(interval(), unexpected=True)],
        [interval(first=0)],
        [interval(first=-1)],
        [interval(last=0)],
        [interval(first=6, last=5)],
        [interval(first=True)],
        [interval(last=True)],
        [interval(first=3.0)],
        [interval(last=5.0)],
        [interval(first="3")],
        [interval(last=None)],
        [interval(bag=None)],
        [interval(bag="")],
        [interval(bag="  ")],
        [interval(reason=None)],
        [interval(reason="")],
        [interval(reason="  ")],
        [interval(), interval(first=False)],
    ],
)
def test_invalid_intervals_fail_before_export(tmp_path, monkeypatch, invalid):
    def unexpected_write(*args):
        pytest.fail("Ungültige Ausschlüsse dürfen keine Bilder schreiben")

    monkeypatch.setattr(preparation.cv2, "imwrite", unexpected_write)
    output = tmp_path / "output"
    with pytest.raises(ValueError, match="Ausschl"):
        preparation.extract(tmp_path, [], output, review=True, exclusions=invalid)
    assert not output.exists()


def test_review_keeps_original_indices_and_inclusive_boundaries(camera_bag, tmp_path):
    exclusions = [interval(1, 1), interval(), interval(8, 8), interval(2, 2, bag="other")]
    output = tmp_path / "output"
    preparation.extract(camera_bag, [], output, review=True, exclusions=exclusions)
    records = json.loads((output.parent.parent / "preparation" / output.name / "review_candidates.json").read_text())
    assert [record["frame"] for record in records] == [2, 6, 7]
    assert sorted(path.name for path in (output / "images").iterdir()) == [
        f"real_recording_{frame:05d}.png" for frame in [2, 6, 7]
    ]
    expected_hash = hashlib.sha256(
        json.dumps(exclusions, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    for record in records:
        frame = record["frame"]
        assert record["id"] == f"real_recording_{frame:05d}"
        assert record["provenance"]["timestamp_ns"] == frame * 100
        assert record["provenance"]["exclusion_method"] == "reviewed_frame_intervals"
        assert record["provenance"]["exclusions_sha256"] == expected_hash
        assert record["boxes"] is None
        assert record["reviewed"] is False
        assert np.all(cv2.imread(str(output / record["image"])) == frame)


@pytest.mark.parametrize("frame", [3, 4, 5])
def test_excluded_annotation_aborts_entire_export_before_writing(tmp_path, monkeypatch, frame):
    def unexpected_write(*args):
        pytest.fail("Konflikt muss vor jeglichem Bildschreiben abbrechen")

    monkeypatch.setattr(preparation.cv2, "imwrite", unexpected_write)
    output = tmp_path / "output"
    with pytest.raises(ValueError, match=f"recording, Frame {frame}.*ausgeschlossen.*Lichtausfall"):
        preparation.extract(tmp_path, [annotation(1), annotation(frame)], output, exclusions=[interval()])
    assert not output.exists()


def test_valid_annotations_preserve_manual_ground_truth(camera_bag, tmp_path):
    output = tmp_path / "output"
    preparation.extract(camera_bag, [annotation(2), annotation(6)], output, exclusions=[interval()])
    records = json.loads((output.parent.parent / "preparation" / output.name / "real_manifest.json").read_text())
    assert [record["frame"] for record in records] == [2, 6]
    assert all(record["boxes"] == annotation(2)["boxes"] for record in records)
    assert all(record["provenance"]["annotation_method"] == "manual" for record in records)


def test_legacy_extract_without_exclusions(camera_bag, tmp_path):
    preparation.extract(camera_bag, [], tmp_path / "output", True)
    records = json.loads((tmp_path.parent / "preparation/output/review_candidates.json").read_text())
    assert [record["frame"] for record in records] == list(range(1, 9))


def test_review_can_exclude_all_frames(camera_bag, tmp_path):
    output = tmp_path / "output"
    preparation.extract(camera_bag, [], output, review=True, exclusions=[interval(1, 8)])
    assert json.loads((output.parent.parent / "preparation" / output.name / "review_candidates.json").read_text()) == []
    assert list((output / "images").iterdir()) == []


@pytest.mark.parametrize("source", ["missing", "default", "explicit"])
def test_cli_loads_optional_exclusions(tmp_path, monkeypatch, source):
    monkeypatch.setattr(preparation, "DIRECTORY", tmp_path)
    arguments = ["prepare_real_dataset.py", str(tmp_path / "bags"), "--extract-for-review"]
    exclusions = [] if source == "missing" else [interval()]
    if source != "missing":
        path = tmp_path / ("custom.json" if source == "explicit" else "real_exclusions.json")
        path.write_text(json.dumps(exclusions))
        if source == "explicit":
            arguments.extend(["--exclusions", str(path)])
    calls = []
    monkeypatch.setattr(sys, "argv", arguments)
    monkeypatch.setattr(preparation, "extract", lambda *args, **kwargs: calls.append((args, kwargs)))
    preparation.main()
    assert len(calls) == 1
    assert calls[0][0][3] is True
    assert calls[0][1]["exclusions"] == exclusions


def test_cli_does_not_ignore_corrupt_exclusions(tmp_path, monkeypatch):
    monkeypatch.setattr(preparation, "DIRECTORY", tmp_path)
    (tmp_path / "real_exclusions.json").write_text("not json")
    monkeypatch.setattr(sys, "argv", ["prepare_real_dataset.py", str(tmp_path), "--extract-for-review"])
    with pytest.raises(json.JSONDecodeError):
        preparation.main()
