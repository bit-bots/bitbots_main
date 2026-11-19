"""Roster reception is informational and never gates referee processing."""

import json
from unittest.mock import Mock

import pytest
from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.node import AutoReferee
from std_msgs.msg import String


def config(setup):
    values = {name: spec.default for name, spec in PARAMETERS.items()}
    values["num_robots"] = setup
    return RefereeConfig.from_parameters(values)


def roster(setup):
    world = pytest.importorskip("bitbots_mujoco_sim.world")
    return [
        {"index": index, "domain": 11 + index, **robot}
        for index, robot in enumerate(world.compute_game_settings(world.parse_num_robots(setup)))
    ]


def test_runtime_roster_is_used_without_comparing_configuration():
    robots = roster("1:3")
    node = Mock(config=config("2:2"), _roster={})
    AutoReferee._on_roster(node, String(data=json.dumps(robots)))
    assert node._roster == {robot["index"]: robot for robot in robots}
    node.create_subscription.assert_not_called()


def test_unchanged_roster_does_not_repeat_events():
    robots = roster("2:2")
    node = Mock(_roster={robot["index"]: robot for robot in robots})
    AutoReferee._on_roster(node, String(data=json.dumps(robots)))
    node._record_event.assert_not_called()


def test_packets_are_sent_before_roster_arrives():
    node = Mock(_roster={})
    AutoReferee._send(node)
    node.adapter.send_state.assert_called_once_with()


def test_opening_sequence_advances_before_roster_arrives():
    node = Mock(_roster={})
    node._startup.finished = False
    node.get_parameter.return_value.value = False
    node._network_clock.now.return_value.nanoseconds = 1
    node._startup.advance.return_value = node.adapter.state
    AutoReferee._advance_startup(node)
    node._startup.advance.assert_called_once_with(node.adapter.state, 1)
