"""Position storage and contact-edge regression cases without a simulator."""

import unittest
from dataclasses import replace
from unittest.mock import Mock

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.rules.check_rules import RuleChecker


def observation(step, touching=(), positions=None, ball=(0.0, 0.0, 0.1)):
    return SimulationObservation(
        time_ns=step * 1_000_000,
        step_number=step,
        ball_position=ball,
        robot_positions=positions if positions is not None else {0: (1.0, 2.0, 0.3)},
        touching_ball=frozenset(touching),
    )


class ObservationTest(unittest.TestCase):
    def setUp(self):
        self.config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
        self.state = MatchState.initial(self.config)
        self.checker = RuleChecker({0: 8, 1: 9})

    def test_positions_are_copied_and_updated_before_callback(self):
        sample = observation(1, touching=(0,))

        def contact(team_id):
            self.assertEqual(team_id, 8)
            self.assertEqual(self.checker.robot_positions, sample.robot_positions)
            self.assertEqual(self.checker.ball_position, sample.ball_position)

        self.checker.on_robot_ball_contact = Mock(side_effect=contact)
        self.assertIs(self.checker.check_rules(self.state, sample), self.state)
        sample.robot_positions.clear()
        self.assertEqual(self.checker.robot_positions, {0: (1.0, 2.0, 0.3)})
        self.checker.on_robot_ball_contact.assert_called_once_with(8)

    def test_contact_fires_once_until_robot_releases_ball(self):
        self.checker.on_robot_ball_contact = Mock()
        for step, touching in ((1, (0,)), (2, (0,)), (3, ()), (4, (0,))):
            self.checker.check_rules(self.state, observation(step, touching))
        self.assertEqual(self.checker.on_robot_ball_contact.call_count, 2)

    def test_each_touching_robot_gets_its_own_team_callback(self):
        self.checker.on_robot_ball_contact = Mock()
        sample = observation(1, touching=(0, 1), positions={0: (0.0, 0.0, 0.0), 1: (1.0, 0.0, 0.0)})
        self.checker.check_rules(self.state, sample)
        self.assertEqual([call.args[0] for call in self.checker.on_robot_ball_contact.call_args_list], [8, 9])

    def test_unknown_team_is_not_guessed(self):
        self.checker.on_robot_ball_contact = Mock()
        self.checker.check_rules(self.state, observation(1, (2,), {2: (1.0, 0.0, 0.0)}))
        self.assertEqual(self.checker.unmapped_robot_indices, {2})
        self.checker.on_robot_ball_contact.assert_not_called()

    def test_removed_objects_do_not_leave_stale_positions(self):
        self.checker.check_rules(self.state, observation(1, (0,)))
        self.checker.check_rules(self.state, observation(2, positions={}, ball=None))
        self.assertIsNone(self.checker.ball_position)
        self.assertEqual(self.checker.robot_positions, {})

    def test_continuous_contact_recovers_last_team_after_opponent_releases(self):
        positions = {0: (1, 0, 0.4), 1: (2, 0, 0.4)}
        for step, touching, expected in ((1, (0,), 8), (2, (0, 1), None), (3, (0,), 8), (4, (), 8)):
            self.checker.check_rules(self.state, observation(step, touching, positions))
            self.assertEqual(self.checker.last_touch_team_id, expected)

    def test_simultaneous_teammate_contacts_are_unambiguous(self):
        checker = RuleChecker({0: 8, 1: 8})
        checker.check_rules(self.state, observation(1, (0, 1), {0: (1, 0, 0.4), 1: (2, 0, 0.4)}))
        self.assertEqual(checker.last_touch_team_id, 8)

    def test_unmapped_continuous_contact_does_not_preserve_a_known_team(self):
        self.checker.check_rules(self.state, observation(1, (0,)))
        self.checker.check_rules(self.state, observation(2, (2,), {2: (1, 0, 0.4)}))
        self.assertIsNone(self.checker.last_touch_team_id)

    def test_robot_teleport_contact_is_suppressed_until_release(self):
        positions = {0: (1, 0, 0.4), 1: (2, 0, 0.4)}
        self.checker.check_rules(self.state, observation(1, (0,), positions))
        teleported = replace(observation(2, (0, 1), positions), teleported_robots=frozenset({1}))
        self.checker.check_rules(self.state, teleported)
        self.assertEqual(self.checker.last_touch_team_id, 8)
        self.checker.check_rules(self.state, observation(3, (0, 1), positions))
        self.assertEqual(self.checker.last_touch_team_id, 8)
        self.checker.check_rules(self.state, observation(4, (0,), positions))
        self.checker.check_rules(self.state, observation(5, (1,), positions))
        self.assertEqual(self.checker.last_touch_team_id, 9)

    def test_ball_teleport_contact_is_suppressed_until_release(self):
        teleported = replace(observation(1, (0,)), ball_teleported=True)
        self.checker.check_rules(self.state, teleported)
        self.checker.check_rules(self.state, observation(2, (0,)))
        self.assertIsNone(self.checker.last_touch_team_id)
        self.checker.check_rules(self.state, observation(3))
        self.checker.check_rules(self.state, observation(4, (0,)))
        self.assertEqual(self.checker.last_touch_team_id, 8)

    def test_simulator_restart_clears_contact_history(self):
        self.checker.check_rules(self.state, observation(10, (0,)))
        self.assertEqual(self.checker.last_touch_team_id, 8)
        self.checker.check_rules(self.state, observation(1))
        self.assertIsNone(self.checker.last_touch_team_id)
        self.checker.check_rules(self.state, observation(2, (0,)))
        self.assertEqual(self.checker.last_touch_team_id, 8)

    def test_instances_do_not_share_positions(self):
        self.checker.check_rules(self.state, observation(1))
        self.assertEqual(RuleChecker({}).robot_positions, {})

    def test_mapping_uses_configured_team_ids(self):
        values = {name: spec.default for name, spec in PARAMETERS.items()}
        values.update(home_team_id=8, away_team_id=9, robot_team_mapping='{"0":"home","1":"away"}')
        self.assertEqual(RefereeConfig.from_parameters(values).robot_teams, {0: 8, 1: 9})

    def test_invalid_mappings_are_rejected(self):
        for mapping in ("[]", '{"-1":"home"}', '{"0":"unknown"}', '{"00":"home"}', '{"0":8}'):
            with self.subTest(mapping=mapping), self.assertRaises(ValueError):
                values = {name: spec.default for name, spec in PARAMETERS.items()}
                values["robot_team_mapping"] = mapping
                RefereeConfig.from_parameters(values)
