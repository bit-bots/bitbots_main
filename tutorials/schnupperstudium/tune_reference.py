"""Reproduzierbarer Parametervergleich auf Trainingsbildern für Betreuende."""

import argparse
import hashlib
import itertools
import json
from pathlib import Path

import cv2
import numpy as np
from werkzeuge import detection_metrics, evaluate, find_dataset_root, load_dataset, match_boxes

COLOR_GRID = ([0, 3, 5], [20, 30, 40], [60, 100, 140], [40, 80])
SHAPE_GRID = ([10, 30, 60], [0, 0.3, 0.5, 0.65, 0.75], [0.4, 0.6, 0.75])
IOU_THRESHOLD = 0.5


def contour_features(hsv, color, opening=False):
    """Berechne Konturmerkmale einmal je Farbbereich."""
    low_h, high_h, saturation, value = color
    mask = cv2.inRange(hsv, np.array([low_h, saturation, value]), np.array([high_h, 255, 255]))
    if opening:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    features = []
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        features.append(([x, y, x + width, y + height], area, 4 * np.pi * area / perimeter**2, width / height))
    return features


def filter_boxes(features, shape):
    """Wende die Formgrenzen an; eine leere Auswahl bleibt ein gültiges Boxarray."""
    min_area, min_roundness, min_aspect = shape
    boxes = [
        box
        for box, area, roundness, aspect in features
        if area >= min_area and roundness >= min_roundness and min(aspect, 1 / aspect) >= min_aspect
    ]
    return np.asarray(boxes, dtype=float).reshape(-1, 4)


def detector(parameters):
    """Erzeuge die im Notebook erklärte Variante ohne Maskenbereinigung."""

    def detect(image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return filter_boxes(contour_features(hsv, parameters[:4]), parameters[4:])

    return detect


def baseline(image):
    """Ursprüngliche Referenz aus dem Branch schnupperstudium2026_vision_full."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    features = contour_features(hsv, (5, 30, 100, 80), opening=True)
    boxes = [
        box for box, area, roundness, aspect in features if area >= 30 and roundness >= 0.75 and 0.75 <= aspect <= 1.33
    ]
    return np.asarray(boxes, dtype=float).reshape(-1, 4)


def summarize(result):
    return {key: value for key, value in result.items() if key != "items"}


def search(records):
    hsv_images = [cv2.cvtColor(record["image_bgr"], cv2.COLOR_BGR2HSV) for record in records]
    best = None
    for color in itertools.product(*COLOR_GRID):
        features = [contour_features(hsv, color) for hsv in hsv_images]
        for shape in itertools.product(*SHAPE_GRID):
            counts = {source: np.zeros(3, dtype=int) for source in ("real", "simulation")}
            for record, candidates in zip(records, features, strict=True):
                boxes = filter_boxes(candidates, shape)
                tp = len(match_boxes(boxes, record["boxes"], threshold=IOU_THRESHOLD))
                counts[record["source"]] += (tp, len(boxes) - tp, len(record["boxes"]) - tp)
            metrics = {source: detection_metrics(*map(int, count)) for source, count in counts.items()}
            score = sum(metric["f1"] for metric in metrics.values()) / len(metrics)
            candidate = {"score": score, "parameters": [*color, *shape], "by_source": metrics}
            if best is None or score > best["score"]:
                best = candidate
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="Pfad für den erzeugten JSON-Bericht")
    parser.add_argument("--evaluate-test", action="store_true", help="Nach Auswahl auch Test und Transfer auswerten")
    args = parser.parse_args()
    records = load_dataset("orange", split="train", limit=None)
    best = search(records)
    print("Beste Trainingsvariante:", best, flush=True)
    chosen = detector(best["parameters"])
    report = {
        "selection": "Mean of real and simulation training F1; first candidate wins ties. No morphology.",
        "parameter_order": ["h_min", "h_max", "s_min", "v_min", "min_area", "min_roundness", "min_aspect"],
        "color_grid": COLOR_GRID,
        "shape_grid": SHAPE_GRID,
        "iou_threshold": IOU_THRESHOLD,
        "selected": best,
        "evaluations": {},
        "manifest_sha256": {},
    }
    datasets = [("orange", "train")]
    if args.evaluate_test:
        datasets += [("orange", "test"), ("realistic", "test")]
    for name, split in datasets:
        manifest = find_dataset_root(name=name) / "datasets" / name / "manifest.json"
        report["manifest_sha256"][name] = hashlib.sha256(manifest.read_bytes()).hexdigest()
        loaded = records if (name, split) == ("orange", "train") else load_dataset(name, split=split, limit=None)
        report["evaluations"][f"{name}/{split}"] = {
            label: summarize(evaluate(loaded, function, threshold=IOU_THRESHOLD))
            for label, function in (("baseline", baseline), ("tuned", chosen))
        }
    args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
