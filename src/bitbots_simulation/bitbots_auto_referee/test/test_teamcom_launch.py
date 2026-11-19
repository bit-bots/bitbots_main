"""Keep AutoRef network overrides opt-in at the simulation entry point."""

import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

BRINGUP = Path(__file__).resolve().parents[3] / "bitbots_misc/bitbots_bringup"


@pytest.fixture
def simulation_launch():
    pytest.importorskip("bitbots_mujoco_sim.world")
    spec = importlib.util.spec_from_file_location("mujoco_launch", BRINGUP / "launch/mujoco_simulation.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("enabled", [False, True])
def test_only_enabled_autoref_changes_the_teamcom_launch(simulation_launch, monkeypatch, tmp_path, enabled):
    module = simulation_launch
    context = LaunchContext()
    context.launch_configurations.update({name: "" for name, _ in module.TEAMPLAYER_ARGS})
    context.launch_configurations.update(
        num_robots="1", robot_type="piplus", web="true", teamplayer="true", auto_referee=str(enabled).lower()
    )
    monkeypatch.setattr(module, "get_package_share_directory", lambda package: str(tmp_path / package))
    monkeypatch.setattr(module, "generate_world_xml", Mock(return_value=tmp_path / "world.xml"))
    monkeypatch.setattr(module, "generate_domain_bridge_config", Mock(return_value=tmp_path / "bridge.yaml"))
    monkeypatch.setattr(module, "compute_game_settings", Mock(return_value=[{}]))
    process = Mock()
    monkeypatch.setattr(module, "ExecuteProcess", process)
    module.launch_setup(context)
    options = [arg for arg in process.call_args.kwargs["cmd"] if arg.startswith("auto_referee:=")]
    assert options == [f"auto_referee:={str(enabled).lower()}"]


def test_autoref_integration_is_disabled_by_default(simulation_launch):
    argument = next(
        action
        for action in simulation_launch.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument) and action.name == "auto_referee"
    )
    assert "".join(value.perform(LaunchContext()) for value in argument.default_value) == "false"


def test_robot_launches_default_to_disabled_autoref_and_forward_the_flag():
    for name in ("teamplayer.launch", "highlevel.launch"):
        root = ET.parse(BRINGUP / "launch" / name).getroot()
        assert root.find("arg[@name='auto_referee']").attrib["default"] == "false"
        assert root.find("arg[@name='teamcom_launch_file']") is None
    teamplayer = ET.parse(BRINGUP / "launch/teamplayer.launch").getroot()
    assert teamplayer.find(".//include/arg[@name='auto_referee']").attrib["value"] == "$(var auto_referee)"


@pytest.mark.parametrize("sim,autoref", [(False, False), (False, True), (True, False), (True, True)])
def test_only_autoref_simulation_selects_the_override(sim, autoref):
    root = ET.parse(BRINGUP / "launch/highlevel.launch").getroot()
    group = root.find("group[@if='$(var teamcom)']")
    values = {"$(var sim)": sim, "$(var auto_referee)": autoref}
    selected = []

    def visit(element):
        if "if" in element.attrib and not values[element.attrib["if"]]:
            return
        if "unless" in element.attrib and values[element.attrib["unless"]]:
            return
        if element.tag == "include":
            selected.append(element.attrib["file"])
        else:
            for child in element:
                visit(child)

    for child in group:
        visit(child)
    assert selected == [
        "$(find-pkg-share bitbots_bringup)/launch/auto_referee_team_comm.launch"
        if sim and autoref
        else "$(find-pkg-share bitbots_team_communication)/launch/team_comm.launch"
    ]


def test_autoref_launch_loads_override_after_the_regular_config():
    root = ET.parse(BRINGUP / "launch/auto_referee_team_comm.launch").getroot()
    files = [param.attrib["from"] for param in root.findall("node/param[@from]")]
    assert files == [
        "$(find-pkg-share bitbots_team_communication)/config/team_communication_config.yaml",
        "$(find-pkg-share bitbots_bringup)/config/auto_referee_team_communication.yaml",
    ]
