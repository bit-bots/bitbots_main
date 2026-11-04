"""Catch identity drift even when both launches use the same total robot count."""

import json
from unittest.mock import Mock

import pytest
from bitbots_auto_referee.adapters.simulation.roster import validate_roster
from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.communication import MessageBudget
from bitbots_auto_referee.core.state import MatchState
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


@pytest.mark.parametrize("setup", ["1", "2:2", "3:1", "0:2", "2:0"])
def test_matching_runtime_roster_is_accepted(setup):
    robots = roster(setup)
    assert validate_roster(config(setup), robots) == {robot["index"]: robot for robot in robots}


def test_equal_totals_with_different_team_assignments_are_rejected():
    with pytest.raises(ValueError, match="assignment mismatch"):
        validate_roster(config("2:2"), roster("1:3"))


@pytest.mark.parametrize("field,value", [("bot_id", 3), ("team_id", 9), ("role", "defense"), ("team_color", 1)])
def test_identity_role_and_palette_drift_are_rejected(field, value):
    robots = roster("2:2")
    robots[0][field] = value
    with pytest.raises(ValueError):
        validate_roster(config("2:2"), robots)


def test_duplicate_domains_are_rejected():
    robots = roster("2:2")
    robots[1]["domain"] = robots[0]["domain"]
    with pytest.raises(ValueError, match="Duplicate"):
        validate_roster(config("2:2"), robots)


def test_invalid_roster_blocks_commands_and_packets_until_configuration_matches():
    node = Mock(config=config("2:2"), _roster_status="waiting", _roster_valid=False)
    AutoReferee._on_roster(node, String(data=json.dumps(roster("1:3"))))
    assert not node._roster_valid
    AutoReferee._send(node)
    AutoReferee._advance_startup(node)
    AutoReferee._on_simulation_step(node, Mock())
    node.adapter.send_state.assert_not_called()
    node._startup.advance.assert_not_called()
    node.rule_checker.check_rules.assert_not_called()


def test_valid_roster_creates_one_statistics_subscription_per_robot():
    node = Mock(config=config("2:2"), _roster_valid=False, _roster={}, _send_subscriptions={})
    AutoReferee._on_roster(node, String(data=json.dumps(roster("2:2"))))
    assert node._roster_valid
    assert [call.args[1] for call in node.create_subscription.call_args_list] == [
        f"/robot{domain}/team_communication/sent" for domain in range(11, 15)
    ]


def test_bridged_report_updates_only_the_verified_senders_team():
    cfg = config("2:2")
    node = Mock(_roster_valid=True, _roster=validate_roster(cfg, roster("2:2")), _budget=MessageBudget())
    node.adapter.state = MatchState.initial(cfg)
    report = {"team_id": cfg.away_team_id, "player_id": 1, "session": "sender", "sent": 7}
    AutoReferee._on_team_message_count(node, 2, String(data=json.dumps(report)))
    updated = node.adapter.set_state.call_args.args[0]
    assert [team.message_budget for team in updated.teams] == [12000, 11993]
    node._send.assert_called_once_with()


def test_report_identity_cannot_charge_the_wrong_team_or_player():
    cfg = config("2:2")
    node = Mock(_roster_valid=True, _roster=validate_roster(cfg, roster("2:2")), _budget=MessageBudget())
    node.adapter.state = MatchState.initial(cfg)
    report = {"team_id": cfg.home_team_id, "player_id": 1, "session": "sender", "sent": 7}
    AutoReferee._on_team_message_count(node, 2, String(data=json.dumps(report)))
    node.adapter.set_state.assert_not_called()
    node._send.assert_not_called()
