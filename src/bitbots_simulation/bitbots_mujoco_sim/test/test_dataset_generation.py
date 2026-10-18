import importlib.util
from pathlib import Path

import mujoco
import numpy as np

GENERATOR_PATH = Path(__file__).resolve().parents[4] / "tutorials/schnupperstudium/generate_sim_dataset.py"
SPEC = importlib.util.spec_from_file_location("generate_sim_dataset", GENERATOR_PATH)
generator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(generator)


def test_segmentation_box_uses_exclusive_pixel_edges():
    mask = np.zeros((20, 30), dtype=bool)
    mask[4:10, 7:18] = True
    assert generator.mask_box(mask) == [7, 4, 18, 10]
    assert generator.mask_box(np.zeros_like(mask)) is None


def test_default_split_counts():
    assert generator.split_counts(100, 0.2) == {"train": 80, "test": 20}


def test_negative_model_has_no_ball_body_geometry_or_joint():
    for positive in (True, False):
        model = mujoco.MjModel.from_xml_string(generator.scene_xml(positive, 320, 240))
        for object_type, name in (
            (mujoco.mjtObj.mjOBJ_BODY, "ball"),
            (mujoco.mjtObj.mjOBJ_GEOM, "ball"),
            (mujoco.mjtObj.mjOBJ_JOINT, "ball-root"),
        ):
            assert (mujoco.mj_name2id(model, object_type, name) >= 0) == positive
        assert model.geom("pitch").id >= 0
        assert model.ncam == 0
