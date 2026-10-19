import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest

XML_DIRECTORY = Path(__file__).resolve().parents[1] / "xml"


def test_launch_defaults_to_orange_and_forwards_ball_style():
    launch = ET.parse(XML_DIRECTORY.parent / "launch/simulator.launch").getroot()
    assert launch.find("arg[@name='ball_style']").get("default") == "orange"
    assert launch.find("node/param[@name='ball_style']").get("value") == "$(var ball_style)"


def compile_scene(style: str, robot_count: int) -> mujoco.MjModel:
    template_name = "kid_field_orange.xml" if style == "orange" else "kid_field.xml"
    template = (XML_DIRECTORY / template_name).read_text()
    template = (
        template.replace("{{NUM_ROBOTS}}", str(robot_count))
        .replace("{{OFFSET}}", "2")
        .replace("{{START_X}}", "1")
        .replace("{{ROBOT_TYPE}}", "piplus")
    )
    assets = {path.name: path.read_bytes() for path in XML_DIRECTORY.glob("*.xml")}
    root = ET.fromstring(assets["kid_field_environment.xml"])
    root.find("compiler").set("assetdir", str(XML_DIRECTORY / "assets"))
    root.find("asset/model").set("file", str(XML_DIRECTORY / "pi_plus.xml"))
    assets["kid_field_environment.xml"] = ET.tostring(root)
    return mujoco.MjModel.from_xml_string(template, assets=assets)


@pytest.mark.parametrize("robot_count", [1, 2])
def test_robot_scenes_preserve_physics_and_camera_interfaces(robot_count):
    original = compile_scene("original", robot_count)
    orange = compile_scene("orange", robot_count)
    for model in (original, orange):
        assert model.joint("ball-root").id >= 0
        assert model.site("B-vismarker").id >= 0
        for index in range(robot_count):
            assert model.body(f"robot_base_link_{index}").id >= 0
            assert model.camera(f"robot_head_camera_{index}").id >= 0
        mujoco.mj_forward(model, mujoco.MjData(model))
    for attribute in (
        "nbody",
        "ngeom",
        "njnt",
        "nu",
        "nsensor",
        "ncam",
        "nq",
        "nv",
        "geom_size",
        "geom_friction",
        "geom_solref",
        "geom_rgba",
        "body_mass",
        "body_inertia",
        "qpos0",
        "actuator_gainprm",
        "sensor_type",
        "cam_pos",
    ):
        np.testing.assert_equal(getattr(original, attribute), getattr(orange, attribute))
    for accessor, count in (
        ("body", original.nbody),
        ("geom", original.ngeom),
        ("joint", original.njnt),
        ("actuator", original.nu),
        ("sensor", original.nsensor),
        ("camera", original.ncam),
        ("site", original.nsite),
    ):
        assert [getattr(original, accessor)(index).name for index in range(count)] == [
            getattr(orange, accessor)(index).name for index in range(count)
        ]
    original_material = original.geom("ball").matid[0]
    orange_material = orange.geom("ball").matid[0]
    assert np.any(original.mat_texid[original_material] >= 0)
    assert np.any(orange.mat_texid[orange_material] >= 0)
    np.testing.assert_allclose(original.mat_rgba[original_material], [1, 1, 1, 1])
    texture = orange.texture("ball")
    start = int(texture.adr[0])
    size = int(texture.width[0] * texture.height[0] * texture.nchannel[0])
    pixels = orange.tex_data[start : start + size].reshape(-1, 3)
    # Both the camera renderer and the Viser overlay sample this cube texture.
    expected = np.broadcast_to([1, 0.32, 0.015], pixels.shape)
    np.testing.assert_allclose(pixels / 255, expected, atol=1 / 255)
