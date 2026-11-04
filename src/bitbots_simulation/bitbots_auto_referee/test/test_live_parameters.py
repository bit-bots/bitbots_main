"""Runtime parameter validation and application without starting network adapters."""

from dataclasses import replace
from unittest.mock import Mock, patch

import pytest
from rclpy.parameter import Parameter

from bitbots_auto_referee.config import LIVE_PARAMETERS, PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.node import AutoReferee
from bitbots_auto_referee.rules.pushing import PushingConfig


def referee():
    values = {name: spec.default for name, spec in PARAMETERS.items()}
    node = Mock()
    node.get_parameter.side_effect = lambda name: Parameter(name, value=values[name])
    node.config = RefereeConfig.from_parameters(values)
    return node


@pytest.mark.parametrize("name", LIVE_PARAMETERS)
@pytest.mark.parametrize("value", [0.0, -1.0, float("nan"), float("inf"), 2, "invalid"])
def test_invalid_runtime_threshold_is_rejected_without_changing_rules(name, value):
    node = referee()
    result = AutoReferee._validate_live_parameters(node, [Parameter(name, value=value)])
    assert not result.successful
    node.rule_checker.pushing_rules.configure.assert_not_called()


def test_atomic_pair_is_validated_together():
    node = referee()
    parameters = [Parameter(name, value=value) for name, value in zip(LIVE_PARAMETERS[:2], (30.0, 40.0), strict=True)]
    assert not AutoReferee._validate_live_parameters(node, parameters[:1]).successful
    assert AutoReferee._validate_live_parameters(node, parameters).successful
    AutoReferee._apply_live_parameters(node, parameters)
    assert node.config.pushing_sustained_force_threshold == 30.0
    assert node.config.pushing_force_threshold == 40.0
    node.rule_checker.pushing_rules.configure.assert_called_once_with(PushingConfig.from_config(node.config))
    node._record_event.assert_called_once()
    node._refresh_dashboard.assert_called_once()


def test_unchanged_thresholds_preserve_pending_contact_evidence():
    node = referee()
    AutoReferee._apply_live_parameters(
        node, [Parameter(name, value=getattr(node.config, name)) for name in LIVE_PARAMETERS]
    )
    node.rule_checker.pushing_rules.configure.assert_not_called()


def test_events_reach_console_and_gamecontroller_dashboard():
    node = referee()
    node.rule_checker.simulation_time_ns = 123
    message = "Pushing: Auslöser: anhaltender Kontakt."
    AutoReferee._record_event(node, message)
    node.get_logger.return_value.info.assert_called_once_with(message)
    node.dashboard.record.assert_called_once_with(message, 123)


def test_live_set_play_distance_updates_only_position_rules():
    node = referee()
    parameters = [Parameter("set_play_min_distance", value=0.8)]
    assert AutoReferee._validate_live_parameters(node, parameters).successful
    AutoReferee._apply_live_parameters(node, parameters)
    assert node.config.set_play_min_distance == 0.8
    node.rule_checker.robot_position_rules.configure_set_play_distance.assert_called_once_with(0.8)
    node.rule_checker.pushing_rules.configure.assert_not_called()
    node._record_event.assert_called_once()
    node._refresh_dashboard.assert_called_once()


@pytest.mark.parametrize("use_sim_time", [True, False])
def test_early_set_transition_updates_opening_clock(use_sim_time):
    node = referee()
    previous = replace(MatchState.initial(node.config), state="STATE_READY", stopped=False)
    updated = replace(previous, state="STATE_SET", secondary_time=5)
    node.adapter.state = previous
    node._roster_valid = True
    node._startup.finished = False
    node.rule_checker.check_rules.return_value = updated
    node.rule_checker.unmapped_robot_indices = set()
    node._unmapped_robot_indices = set()
    node.get_parameter.side_effect = lambda name: Parameter(name, value=use_sim_time)
    node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    node._network_clock.now.return_value.nanoseconds = 20_000_000_000
    with patch("bitbots_auto_referee.node.decode_observation", return_value=Mock()):
        AutoReferee._on_simulation_step(node, Mock())
    node._startup.finish_ready.assert_called_once_with(10_000_000_000 if use_sim_time else 20_000_000_000)
    node.adapter.set_state.assert_called_once_with(updated)
