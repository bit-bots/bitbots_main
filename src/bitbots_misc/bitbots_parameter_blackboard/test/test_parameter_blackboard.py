from pathlib import Path

import cv2
import numpy as np
import pytest
import rclpy
from bitbots_utils.utils import (
    MissingGlobalParameterError,
    get_global_parameters,
    wait_for_latched_message,
)
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter

from bitbots_parameter_blackboard.parameter_blackboard import (
    FIELD_MAP_MAX_INTENSITY,
    ParameterBlackboard,
    config_directory,
    load_field_map,
    load_parameter_files,
    occupancy_grid_from_field_map,
    parameter_files,
)

FIELD_NAME = "labor"


def write_yaml(path: Path, content: str) -> Path:
    path.write_text(content)
    return path


def test_later_parameter_files_take_precedence(tmp_path: Path):
    first = write_yaml(
        tmp_path / "first.yaml",
        "parameter_blackboard:\n  ros__parameters:\n    bot_id: 1\n    field:\n      size:\n        x: 5.0\n",
    )
    second = write_yaml(tmp_path / "second.yaml", "parameter_blackboard:\n  ros__parameters:\n    bot_id: 2\n")

    parameters = load_parameter_files([first, second])

    assert parameters["bot_id"].value == 2
    assert parameters["field.size.x"].value == 5.0


def test_missing_parameter_file_raises(tmp_path: Path):
    with pytest.raises(FileNotFoundError):
        load_parameter_files([tmp_path / "does_not_exist.yaml"])


def test_sim_game_settings_are_only_loaded_in_simulation(tmp_path: Path):
    assert len(parameter_files(tmp_path, FIELD_NAME, False, "12")) == len(parameter_files(tmp_path, FIELD_NAME, True, None))
    assert parameter_files(tmp_path, FIELD_NAME, True, "12")[-1].name == "sim_game_settings_12.yaml"


def test_field_map_rejects_intensities_beyond_expected_range(tmp_path: Path):
    path = tmp_path / "lines.png"
    cv2.imwrite(str(path), np.array([[0, FIELD_MAP_MAX_INTENSITY + 1]], dtype=np.uint8))

    with pytest.raises(ValueError, match="lines.png"):
        load_field_map(path)


def test_field_map_is_converted_to_centered_occupancy_grid(tmp_path: Path):
    path = tmp_path / "lines.png"
    image = np.array([[0, 25, 50], [75, 100, 100]], dtype=np.uint8)
    cv2.imwrite(str(path), image)

    msg = occupancy_grid_from_field_map(load_field_map(path), "map")

    assert (msg.info.height, msg.info.width) == image.shape
    assert list(msg.data) == (FIELD_MAP_MAX_INTENSITY - image.astype(int)).ravel().tolist()
    assert msg.info.origin.position.x == pytest.approx(-msg.info.width / 2 * msg.info.resolution)
    assert msg.info.origin.position.y == pytest.approx(-msg.info.height / 2 * msg.info.resolution)


@pytest.fixture
def nodes():
    rclpy.init()
    blackboard = ParameterBlackboard(
        parameter_overrides=[
            Parameter("field.name", value=FIELD_NAME),
            Parameter("simulation_active", value=False),
        ]
    )
    client = Node("parameter_blackboard_test_client")
    yield blackboard, client
    client.destroy_node()
    blackboard.destroy_node()
    rclpy.shutdown()


def test_blackboard_publishes_loaded_configuration(nodes):
    _, client = nodes
    expected = load_parameter_files(parameter_files(config_directory(), FIELD_NAME, False, None))

    received = get_global_parameters(client, list(expected.keys()) + ["field.name"], timeout_sec=10.0)

    assert received["field.name"] == FIELD_NAME
    for name, parameter in expected.items():
        assert received[name] == parameter.value


def test_blackboard_republishes_parameter_changes(nodes):
    blackboard, client = nodes
    new_bot_id = get_global_parameters(client, ["bot_id"], timeout_sec=10.0)["bot_id"] + 1

    assert blackboard.set_parameters([Parameter("bot_id", value=new_bot_id)])[0].successful

    assert get_global_parameters(client, ["bot_id"], timeout_sec=10.0)["bot_id"] == new_bot_id


def test_blackboard_rejects_field_change(nodes):
    blackboard, _ = nodes

    assert not blackboard.set_parameters([Parameter("field.name", value="other_field")])[0].successful


def test_missing_global_parameter_raises(nodes):
    _, client = nodes

    with pytest.raises(MissingGlobalParameterError):
        get_global_parameters(client, ["this.parameter.does.not.exist"], timeout_sec=10.0)


def test_blackboard_publishes_lines_map(nodes):
    _, client = nodes
    image = cv2.imread(str(config_directory() / "fields" / FIELD_NAME / "lines.png"), cv2.IMREAD_GRAYSCALE)

    msg = wait_for_latched_message(client, OccupancyGrid, "field/map/lines", timeout_sec=10.0)

    assert (msg.info.height, msg.info.width) == image.shape
