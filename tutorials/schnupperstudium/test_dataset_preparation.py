import ast
import json
from pathlib import Path

import cv2
import numpy as np
import pytest
from assemble_orange_dataset import assemble
from prepare_real_dataset import extract
from werkzeuge import box_als_array, zu_hsv


def dataset(directory, records):
    (directory / "images").mkdir()
    cv2.imwrite(str(directory / "images/example.png"), np.zeros((20, 30, 3), dtype=np.uint8))
    preparation = directory.parent.parent / "preparation" / directory.name
    preparation.mkdir(parents=True, exist_ok=True)
    (preparation / "real_manifest.json").write_text("[]")
    (preparation / "sim_manifest.json").write_text(json.dumps(records))


def example():
    return {
        "id": "example",
        "image": "images/example.png",
        "boxes": [[1, 2, 10, 12]],
        "source": "simulation",
        "split": "train",
    }


def test_incomplete_dataset_requires_explicit_opt_in(tmp_path):
    dataset(tmp_path, [example()])
    with pytest.raises(ValueError, match="unvollständig"):
        assemble(tmp_path)
    assert not (tmp_path / "manifest.json").exists()
    assert (
        json.loads((tmp_path.parent.parent / "preparation" / tmp_path.name / "status.json").read_text())["complete"]
        is False
    )
    assemble(tmp_path, allow_incomplete=True)
    assert len(json.loads((tmp_path / "manifest.json").read_text())) == 1


@pytest.mark.parametrize("boxes", [[[0, 0, 31, 10]], [[5, 0, 2, 10]], [[0, 0, float("nan"), 10]]])
def test_invalid_annotation_is_rejected(tmp_path, boxes):
    record = example()
    record["boxes"] = boxes
    dataset(tmp_path, [record])
    with pytest.raises(ValueError, match="Ungültige Box"):
        assemble(tmp_path, allow_incomplete=True)


def test_duplicate_record_is_rejected(tmp_path):
    dataset(tmp_path, [example(), example()])
    with pytest.raises(ValueError, match="Doppelter"):
        assemble(tmp_path, allow_incomplete=True)


def test_unreviewed_empty_annotations_do_not_export(tmp_path):
    target = tmp_path / "output"
    with pytest.raises(ValueError, match="keine geprüften Annotationen"):
        extract(tmp_path, [], target)
    assert not target.exists()


def test_notebook_result_adapts_to_ros_box_array():
    directory = Path(__file__).resolve().parent
    notebook = json.loads((directory / "loesung.ipynb").read_text())
    functions = [
        node
        for cell in notebook["cells"]
        if cell["cell_type"] == "code"
        for node in ast.parse("".join(cell["source"])).body
        if isinstance(node, ast.FunctionDef) and node.name == "detect_balls"
    ]
    assert len(functions) == 1
    namespace = {"np": np, "zu_hsv": zu_hsv}
    exec(compile(ast.Module(body=functions, type_ignores=[]), "notebook", "exec"), namespace)
    detect = namespace["detect_balls"]
    image = np.zeros((80, 100, 3), dtype=np.uint8)
    assert box_als_array(detect(image)).shape == (0, 4)
    image[20:50, 30:60] = (0, 140, 255)
    np.testing.assert_array_equal(box_als_array(detect(image)), [[30, 20, 60, 50]])
