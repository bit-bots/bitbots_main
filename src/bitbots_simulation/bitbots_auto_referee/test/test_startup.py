"""Opening-sequence regression cases with explicit clock samples, without sleeping."""

import unittest
from dataclasses import replace

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.rules.startup import StartupSequence


class StartupTest(unittest.TestCase):
    def setUp(self):
        config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
        self.state = MatchState.initial(config)
        self.sequence = StartupSequence()
        self.origin = 100_000_000_000
        self.sequence.advance(self.state, self.origin)

    def test_phase_boundaries_and_countdowns(self):
        cases = (
            (0, "STATE_INITIAL", True, 0),
            (4_999_999_999, "STATE_INITIAL", True, 0),
            (5_000_000_000, "STATE_READY", False, 25),
            (29_999_999_999, "STATE_READY", False, 1),
            (30_000_000_000, "STATE_SET", False, 5),
            (34_999_999_999, "STATE_SET", False, 1),
            (35_000_000_000, "STATE_PLAYING", False, 0),
        )
        for elapsed, phase, stopped, remaining in cases:
            with self.subTest(elapsed=elapsed):
                self.state = self.sequence.advance(self.state, self.origin + elapsed)
                self.assertEqual(self.state.state, phase)
                self.assertEqual(self.state.stopped, stopped)
                self.assertEqual(self.state.secondary_time, remaining)
                self.assertEqual(self.state.secs_remaining, 600)
        self.assertTrue(self.sequence.finished)

    def test_paused_clock_does_not_advance(self):
        ready = self.sequence.advance(self.state, self.origin + 5_000_000_000)
        self.assertEqual(self.sequence.advance(ready, self.origin + 5_000_000_000), ready)

    def test_delayed_callback_uses_elapsed_time(self):
        state = self.sequence.advance(self.state, self.origin + 32_000_000_000)
        self.assertEqual(state.state, "STATE_SET")
        self.assertEqual(state.secondary_time, 3)

    def test_backward_clock_jump_restarts_preparation(self):
        ready = self.sequence.advance(self.state, self.origin + 6_000_000_000)
        restarted = self.sequence.advance(ready, 1_000_000_000)
        self.assertEqual(restarted.state, "STATE_INITIAL")
        self.assertEqual(self.sequence.advance(restarted, 6_000_000_000).state, "STATE_READY")

    def test_empty_teams_do_not_block_start(self):
        empty = replace(self.state, teams=tuple(replace(team, players=()) for team in self.state.teams))
        playing = self.sequence.advance(empty, self.origin + 35_000_000_000)
        self.assertEqual(playing.state, "STATE_PLAYING")
        self.assertEqual(playing.teams, empty.teams)

    def test_finished_sequence_does_not_override_later_rules(self):
        playing = self.sequence.advance(self.state, self.origin + 35_000_000_000)
        finished = replace(playing, state="STATE_FINISHED", stopped=True)
        self.assertIs(self.sequence.advance(finished, self.origin + 40_000_000_000), finished)
