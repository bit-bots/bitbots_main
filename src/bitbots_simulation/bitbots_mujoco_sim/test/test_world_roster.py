"""The generated world retains the same identities used to launch the robot stacks."""

import json
import xml.etree.ElementTree as ET
from unittest.mock import patch

import mujoco
from bitbots_mujoco_sim.world import compute_game_settings, generate_world_xml


def test_generated_world_embeds_the_complete_roster(tmp_path):
    xml = tmp_path / "xml"
    xml.mkdir()
    (xml / "kid_field.xml").write_text('<mujoco><worldbody><replicate count="1"/></worldbody></mujoco>')
    with patch("bitbots_mujoco_sim.world.get_jersey_texture", return_value=tmp_path / "jersey.png"):
        world = generate_world_xml("2:2", str(tmp_path), "piplus")
    text = ET.parse(world).getroot().find("./custom/text[@name='simulation_roster']")
    robots = json.loads(text.attrib["data"])
    assert robots == [{"index": index, **setting} for index, setting in enumerate(compute_game_settings([2, 2]))]
    root = ET.Element("mujoco")
    custom = ET.SubElement(root, "custom")
    custom.append(text)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    assert model.ntext == 1
