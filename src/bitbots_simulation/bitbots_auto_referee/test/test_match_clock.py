"""Playing-clock accounting with synthetic simulation time and no sleeping."""

import unittest
from dataclasses import replace
from unittest.mock import Mock

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.rules.check_rules import RuleChecker


class MatchClockTest(unittest.TestCase):
    def setUp(self):
        config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
        self.state = replace(MatchState.initial(config), state="STATE_PLAYING", stopped=False, secs_remaining=10)
        self.events = Mock()
        self.checker = RuleChecker({}, self.events)
        self.step = 0

    def advance(self, seconds):
        self.step += 1
        observation = SimulationObservation(int(seconds * 1_000_000_000), self.step, None, {}, frozenset())
        self.state = self.checker.check_rules(self.state, observation)

    def test_fractional_updates_accumulate_without_rounding_each_step(self):
        for time in (0, 0.4, 0.8):
            self.advance(time)
            self.assertEqual(self.state.secs_remaining, 10)
        self.advance(1.2)
        self.assertEqual(self.state.secs_remaining, 9)
        self.advance(2.0)
        self.assertEqual(self.state.secs_remaining, 8)

    def test_stopped_intervals_preserve_fraction_without_counting_pause(self):
        self.advance(0)
        self.advance(0.4)
        self.state = replace(self.state, stopped=True)
        self.advance(0.4)
        self.advance(100)
        self.assertEqual(self.state.secs_remaining, 10)
        self.state = replace(self.state, stopped=False)
        self.advance(100)
        self.advance(100.6)
        self.assertEqual(self.state.secs_remaining, 9)

    def test_non_playing_phases_do_not_count_down(self):
        for phase in ("STATE_INITIAL", "STATE_READY", "STATE_SET", "STATE_FINISHED"):
            with self.subTest(phase=phase):
                self.state = replace(self.state, state=phase, stopped=False)
                self.advance(100)
                self.advance(200)
                self.assertEqual(self.state.secs_remaining, 10)

    def test_paused_simulation_does_not_count_down(self):
        self.advance(0)
        self.advance(0)
        self.assertEqual(self.state.secs_remaining, 10)

    def test_entering_playing_does_not_charge_preparation_time(self):
        self.state = replace(self.state, state="STATE_SET")
        self.advance(0)
        self.state = replace(self.state, state="STATE_PLAYING")
        self.advance(25)
        self.assertEqual(self.state.secs_remaining, 10)
        self.advance(26)
        self.assertEqual(self.state.secs_remaining, 9)

    def test_clock_reset_rebases_without_restoring_elapsed_seconds(self):
        self.advance(100)
        self.advance(101.5)
        self.advance(0)
        self.advance(0.5)
        self.assertEqual(self.state.secs_remaining, 9)

    def test_external_clock_adjustment_clears_old_fraction(self):
        self.advance(0)
        self.advance(0.9)
        self.state = replace(self.state, secs_remaining=20)
        self.advance(1)
        self.advance(1.2)
        self.assertEqual(self.state.secs_remaining, 20)

    def test_time_stays_at_zero_and_expiry_is_reported_once(self):
        self.advance(0)
        self.advance(20)
        self.advance(30)
        self.assertEqual(self.state.secs_remaining, 0)
        self.assertEqual(self.state.state, "STATE_PLAYING")
        self.events.assert_called_once()
