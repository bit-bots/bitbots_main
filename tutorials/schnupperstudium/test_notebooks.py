"""Offline-Prüfungen für die Lernschritte und die Referenzfunktion."""

import ast
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

DIRECTORY = Path(__file__).resolve().parent


def load_detector(name="loesung.ipynb"):
    """Lade nur die markierten Rechenschritte, ohne Daten oder Zeichenhilfen auszuführen."""
    notebook = json.loads((DIRECTORY / name).read_text())
    namespace = {"cv2": cv2, "np": np}
    for cell in notebook["cells"]:
        if "detector" not in cell["metadata"].get("tags", []):
            continue
        tree = ast.parse("".join(cell["source"]))
        tree.body = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.Assign))]
        exec(compile(tree, name, "exec"), namespace)
    return namespace


@pytest.mark.parametrize("name", ["aufgabe.ipynb", "loesung.ipynb"])
def test_empty_image_and_input_unchanged(name):
    namespace = load_detector(name)
    image = np.zeros((80, 100, 3), dtype=np.uint8)
    original = image.copy()
    boxes = namespace["detect_balls"](image)
    assert boxes.shape == (0, 4)
    np.testing.assert_array_equal(image, original)


def test_solution_finds_separate_balls_without_modifying_image():
    namespace = load_detector()
    hsv = np.zeros((100, 180, 3), dtype=np.uint8)
    for center in ((40, 45), (130, 50)):
        cv2.circle(hsv, center, 15, (10, 220, 180), -1)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    original = image.copy()
    boxes = namespace["detect_balls"](image)
    assert sorted(boxes.tolist()) == [[25, 30, 56, 61], [115, 35, 146, 66]]
    np.testing.assert_array_equal(image, original)


def test_mask_and_detector_use_edited_color_limits():
    namespace = load_detector()
    hsv = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.circle(hsv, (50, 50), 15, (10, 220, 180), -1)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    assert namespace["make_mask"](image).any()
    assert len(namespace["detect_balls"](image)) == 1
    namespace["hsv_unten"] = np.array([70, 140, 80])
    namespace["hsv_oben"] = np.array([90, 255, 255])
    assert not namespace["make_mask"](image).any()
    assert namespace["detect_balls"](image).shape == (0, 4)
