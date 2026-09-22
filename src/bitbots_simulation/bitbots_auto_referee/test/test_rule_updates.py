"""Check the simulation-step callback without starting ROS or networking."""

import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from bitbots_msgs.msg import SimulationState

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.node import AutoReferee
from bitbots_auto_referee.rules.check_rules import RuleChecker


class RuleUpdateTest(unittest.TestCase):
    def setUp(self):
        config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
        self.state = MatchState.initial(config)
        self.node = Mock()
        self.node.adapter.state = self.state
        self.node.rule_checker = RuleChecker(config.robot_teams)
        self.node._unmapped_robot_indices = set()
        self.message = SimulationState()
        self.message.header.stamp.sec = 12
        self.message.header.stamp.nanosec = 345

    def test_unchanged_result_keeps_state_and_heartbeat_schedule(self):
        AutoReferee._on_simulation_step(self.node, self.message)
        self.node.adapter.set_state.assert_not_called()
        self.node._send.assert_not_called()

    def test_each_update_checks_rules_in_every_phase(self):
        for phase in ("STATE_INITIAL", "STATE_READY", "STATE_SET", "STATE_PLAYING", "STATE_FINISHED"):
            with self.subTest(phase=phase):
                state = replace(self.state, state=phase)
                self.node.adapter.state = state
                with patch.object(self.node.rule_checker, "check_rules", return_value=state) as check:
                    AutoReferee._on_simulation_step(self.node, self.message)
                    AutoReferee._on_simulation_step(self.node, self.message)
                    self.assertEqual(check.call_count, 2)
                    self.assertIs(check.call_args.args[0], state)
                    self.assertEqual(check.call_args.args[1].time_ns, 12_000_000_345)

    def test_rule_change_is_applied_and_sent_immediately(self):
        updated = replace(self.state, state="STATE_FINISHED")
        with patch.object(self.node.rule_checker, "check_rules", return_value=updated):
            AutoReferee._on_simulation_step(self.node, self.message)
        self.node.adapter.set_state.assert_called_once_with(updated)
        self.node._send.assert_called_once_with()
