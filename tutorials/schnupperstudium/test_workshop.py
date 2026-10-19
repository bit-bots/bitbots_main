"""Offline-Tests für faire Detektionsauswertung und das Manifestformat."""

import json

import cv2
import numpy as np
import pytest
from werkzeuge import as_boxes, box_iou, evaluate, load_dataset, match_boxes


def record(boxes, source="real", identifier="bild"):
    return {"id": identifier, "boxes": boxes, "source": source, "image_bgr": np.zeros((24, 24, 3), np.uint8)}


def test_iou_geometry():
    np.testing.assert_allclose(
        box_iou([[0, 0, 10, 10]], [[0, 0, 10, 10], [5, 0, 15, 10], [10, 0, 20, 10]]), [[1, 1 / 3, 0]]
    )


def test_iou_empty_shapes():
    assert box_iou([], []).shape == (0, 0)
    assert box_iou([[0, 0, 1, 1]], []).shape == (1, 0)
    assert box_iou([], [[0, 0, 1, 1]]).shape == (0, 1)


@pytest.mark.parametrize("boxes", [[[1, 0, 0, 1]], [[0, 0, 0, 1]], [[0, 0, np.nan, 1]], [0, 0, 1, 1], np.empty((0, 3))])
def test_invalid_boxes(boxes):
    with pytest.raises(ValueError):
        as_boxes(boxes)


@pytest.mark.parametrize("threshold", [0, -0.1, 1.1, np.nan])
def test_invalid_threshold(threshold):
    with pytest.raises(ValueError):
        evaluate([], lambda image: [], threshold)


def test_duplicate_prediction_counts_as_false_positive():
    result = evaluate([record([[0, 0, 10, 10]])], lambda image: [[0, 0, 10, 10]] * 2)
    assert result["overall"] == {"tp": 1, "fp": 1, "fn": 0, "precision": 0.5, "recall": 1, "f1": 2 / 3}


def test_one_prediction_cannot_match_multiple_balls():
    matches = match_boxes([[0, 0, 10, 10]], [[0, 0, 10, 10], [0, 0, 10, 10]])
    assert len(matches) == 1


def test_matching_reassigns_instead_of_losing_possible_match():
    predicted = [[2, 0, 12, 10], [0, 0, 10, 10]]
    truth = [[0, 0, 10, 10], [5, 0, 15, 10]]
    assert match_boxes(predicted, truth) == [(0, 1), (1, 0)]


def test_threshold_is_inclusive():
    assert match_boxes([[0, 0, 10, 10]], [[0, 0, 5, 10]], threshold=0.5) == [(0, 0)]


def test_negative_image_has_no_invented_ground_truth():
    result = evaluate([record([])], lambda image: [[0, 0, 5, 5]])
    assert (result["overall"]["tp"], result["overall"]["fp"], result["overall"]["fn"]) == (0, 1, 0)


def test_empty_detector_and_source_aggregation():
    result = evaluate([record([[0, 0, 5, 5]]), record([], "simulation")], lambda image: np.empty((0, 4)))
    assert result["overall"]["fn"] == 1
    assert result["by_source"]["real"]["fn"] == 1
    assert result["by_source"]["simulation"]["fn"] == 0
    assert all(result["overall"][name] == 0 for name in ("precision", "recall", "f1", "tp", "fp"))


def test_empty_evaluation_is_finite():
    result = evaluate([], lambda image: [])
    assert all(value == 0 for value in result["overall"].values())
    assert set(result["by_source"]) == {"real", "simulation"}


def test_missed_and_unmatched_boxes():
    result = evaluate([record([[0, 0, 5, 5]])], lambda image: [[10, 10, 15, 15]])
    assert (result["overall"]["tp"], result["overall"]["fp"], result["overall"]["fn"]) == (0, 1, 1)


def test_micro_average_and_sources():
    records = [record([[0, 0, 5, 5]], identifier="positiv"), record([], "simulation", "negativ")]
    result = evaluate(records, lambda image: [[0, 0, 5, 5]])
    assert result["overall"]["precision"] == 0.5
    assert result["by_source"]["real"]["precision"] == 1
    assert result["by_source"]["simulation"]["fp"] == 1


def test_loader_relative_image_and_empty_ground_truth(tmp_path):
    folder = tmp_path / "datasets" / "orange"
    (folder / "images").mkdir(parents=True)
    cv2.imwrite(str(folder / "images" / "test.jpg"), np.zeros((24, 24, 3), np.uint8))
    manifest = [
        {"id": "train", "image": "images/test.jpg", "boxes": [], "source": "real", "split": "train"},
        {"id": "test", "image": "images/test.jpg", "boxes": [[0, 0, 5, 5]], "source": "simulation", "split": "test"},
    ]
    (folder / "manifest.json").write_text(json.dumps(manifest))
    loaded = load_dataset(root=tmp_path)
    assert len(loaded) == 1
    assert loaded[0]["id"] == "train"
    assert loaded[0]["boxes"].shape == (0, 4)
    assert loaded[0]["image_bgr"].shape == (24, 24, 3)


def test_missing_manifest_is_helpful(tmp_path):
    with pytest.raises(FileNotFoundError, match="Workshop-Daten"):
        load_dataset(root=tmp_path)


@pytest.mark.parametrize("invalid", ["../outside.jpg", "/tmp/outside.jpg"])
def test_loader_rejects_paths_outside_dataset(tmp_path, invalid):
    folder = tmp_path / "datasets" / "orange"
    folder.mkdir(parents=True)
    manifest = [{"id": "bild", "image": invalid, "boxes": [], "source": "real", "split": "train"}]
    (folder / "manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="innerhalb"):
        load_dataset(root=tmp_path)


def test_missing_image_is_helpful(tmp_path):
    folder = tmp_path / "datasets" / "orange"
    folder.mkdir(parents=True)
    manifest = [{"id": "bild", "image": "images/fehlt.jpg", "boxes": [], "source": "real", "split": "train"}]
    (folder / "manifest.json").write_text(json.dumps(manifest))
    with pytest.raises(FileNotFoundError, match="Bild fehlt"):
        load_dataset(root=tmp_path)


def test_balanced_subset_and_limit(tmp_path):
    folder = tmp_path / "datasets" / "orange"
    (folder / "images").mkdir(parents=True)
    cv2.imwrite(str(folder / "images" / "test.jpg"), np.zeros((24, 24, 3), np.uint8))
    manifest = [
        {
            "id": f"{source}-{positive}-{index}",
            "image": "images/test.jpg",
            "boxes": [[0, 0, 5, 5]] if positive else [],
            "source": source,
            "split": "train",
        }
        for source in ("real", "simulation")
        for positive in (True, False)
        for index in range(3)
    ]
    (folder / "manifest.json").write_text(json.dumps(manifest))
    loaded = load_dataset(root=tmp_path, limit=4)
    assert len({(item["source"], bool(len(item["boxes"]))) for item in loaded}) == 4
    assert len(load_dataset(root=tmp_path, limit=None)) == len(manifest)


def test_detector_receives_copy_of_image():
    sample = record([])

    def mutating_detector(image):
        image[:] = 255
        return []

    evaluate([sample], mutating_detector)
    assert not sample["image_bgr"].any()
