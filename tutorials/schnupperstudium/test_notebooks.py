"""Offline-Prüfungen der Histogramm-Notebooks und ihrer Roboterschnittstelle."""

import ast
import json
from pathlib import Path

import cv2
import numpy as np
import pytest
from werkzeuge import ball_verdoppeln, box_als_array, detektor_anzeigen, zu_hsv

DIRECTORY = Path(__file__).resolve().parent


def notebook_functions(name):
    notebook = json.loads((DIRECTORY / name).read_text())
    namespace = {"zu_hsv": zu_hsv, "np": np}
    for cell in notebook["cells"]:
        if cell["cell_type"] != "code":
            continue
        tree = ast.parse("".join(cell["source"]))
        tree.body = [node for node in tree.body if isinstance(node, ast.FunctionDef)]
        exec(compile(tree, name, "exec"), namespace)
    return namespace


def orange_image(height=100, width=120):
    hsv = np.zeros((height, width, 3), dtype=np.uint8)
    hsv[20:50, 40:70] = (15, 220, 180)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR_FULL)


@pytest.mark.parametrize("name", ["evaluation.ipynb", "loesung.ipynb"])
def test_no_ball_returns_none(name):
    detect = notebook_functions(name)["detect_balls"]
    image = np.zeros((80, 100, 3), dtype=np.uint8)
    boxes = detect(image)
    assert boxes is None


def test_solution_rectangle_uses_exclusive_bounds_and_preserves_input():
    detect = notebook_functions("loesung.ipynb")["detect_balls"]
    image = orange_image()
    original = image.copy()
    assert detect(image) == (40, 20, 70, 50)
    np.testing.assert_array_equal(image, original)


@pytest.mark.parametrize("top,left", [(0, 0), (0, 90), (70, 0), (70, 90)])
def test_solution_handles_all_image_edges(top, left):
    hsv = np.zeros((100, 120, 3), dtype=np.uint8)
    hsv[top : top + 30, left : left + 30] = (15, 220, 180)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR_FULL)
    boxes = notebook_functions("loesung.ipynb")["detect_balls"](image)
    assert boxes == (left, top, left + 30, top + 30)


def test_sparse_markings_do_not_become_a_ball():
    hsv = np.zeros((80, 100, 3), dtype=np.uint8)
    hsv[20:22, 30:32] = (15, 220, 180)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR_FULL)
    assert notebook_functions("loesung.ipynb")["detect_balls"](image) is None


def test_histogram_counts_every_pixel():
    histogram = notebook_functions("loesung.ipynb")["histogramm"]
    channel = np.array([[0, 255, 8], [8, 8, 255]], dtype=np.uint8)
    result = histogram(channel)
    assert sum(result) == channel.size
    assert (result[0], result[8], result[255]) == (1, 3, 2)


def test_rgb_exercises_and_bgr_detector_use_identical_hsv():
    bgr = orange_image()
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    np.testing.assert_array_equal(zu_hsv(rgb), zu_hsv(bgr, bgr=True))
    np.testing.assert_array_equal(zu_hsv(bgr, bgr=True), cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV_FULL))


def test_display_adapter_passes_bgr_to_detector(monkeypatch):
    bgr = orange_image()
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    monkeypatch.setattr("werkzeuge.bild_anzeigen", lambda *args, **kwargs: None)

    def detector(image):
        np.testing.assert_array_equal(image, bgr)
        return None

    assert detektor_anzeigen(rgb, detector) is None


def test_bonus_radius_handles_missing_center_and_all_edges():
    radius = notebook_functions("loesung.ipynb")["ball_radius"]
    mask = np.ones((31, 41), dtype=bool)
    assert radius(mask, None, None) == 0
    for x, y in ((0, 0), (40, 0), (0, 30), (40, 30)):
        assert radius(mask, x, y) == 0
    assert radius(mask, 20, 15) == 15


def test_copying_ball_clips_at_image_edges_without_modifying_input():
    image = np.zeros((10, 12, 3), dtype=np.uint8)
    image[5, 10] = (200, 100, 0)
    mask = np.zeros((10, 12), dtype=bool)
    mask[5, 10] = True
    original = image.copy()
    np.testing.assert_array_equal(ball_verdoppeln(image, mask, 10, 0), image)
    copied = ball_verdoppeln(image, mask, -10, 0)
    np.testing.assert_array_equal(copied[5, 0], image[5, 10])
    np.testing.assert_array_equal(image, original)


def test_evaluation_adapter_supports_one_box_and_none():
    assert box_als_array(None).shape == (0, 4)
    np.testing.assert_array_equal(box_als_array((10, 20, 30, 40)), [[10, 20, 30, 40]])


@pytest.mark.parametrize("box", [[10, 20, 30, 40], ((1, 2, 3, 4),), (10, 20, 5, 40), (1, 2, np.nan, 4)])
def test_evaluation_adapter_rejects_invalid_output(box):
    with pytest.raises(ValueError):
        box_als_array(box)


def test_student_notebook_does_not_contain_detector_or_evaluation():
    notebook = json.loads((DIRECTORY / "aufgabe.ipynb").read_text())
    sources = "\n".join("".join(cell["source"]) for cell in notebook["cells"] if cell["cell_type"] == "code")
    assert "detect_balls" not in sources
    assert "auswerten" not in sources
    assert "sys.path" not in sources


def test_evaluation_imports_support_the_completed_detector():
    notebook = json.loads((DIRECTORY / "evaluation.ipynb").read_text())
    namespace = {}
    for cell in notebook["cells"]:
        if cell["cell_type"] != "code":
            continue
        tree = ast.parse("".join(cell["source"]))
        tree.body = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))]
        exec(compile(tree, "evaluation.ipynb", "exec"), namespace)
    solution = json.loads((DIRECTORY / "loesung.ipynb").read_text())
    detector = next(
        "".join(cell["source"]) for cell in solution["cells"] if "".join(cell["source"]).startswith("def detect_balls")
    )
    exec(detector, namespace)
    assert namespace["detect_balls"](orange_image()) == (40, 20, 70, 50)


def test_ros_detector_matches_notebook_reference():
    from schnupper_vision.detector import detect_balls
    from schnupper_vision.detector import zu_hsv as ros_zu_hsv

    notebook_detect = notebook_functions("loesung.ipynb")["detect_balls"]
    image = orange_image()
    assert detect_balls(image) == notebook_detect(image)
    assert detect_balls(np.zeros_like(image)) is None
    np.testing.assert_array_equal(ros_zu_hsv(image, bgr=True), zu_hsv(image, bgr=True))
