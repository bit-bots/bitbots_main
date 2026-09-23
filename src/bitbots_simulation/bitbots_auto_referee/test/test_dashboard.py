"""Dashboard data transport and display formatting without opening a window."""

import json
import unittest
from dataclasses import replace
from unittest.mock import Mock

from std_msgs.msg import String

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.ui.application import DashboardSubscriber, format_snapshot
from bitbots_auto_referee.ui.dashboard import Dashboard


class DashboardTest(unittest.TestCase):
    def setUp(self):
        config = RefereeConfig.from_parameters({name: spec.default for name, spec in PARAMETERS.items()})
        self.state = replace(MatchState.initial(config), state="STATE_PLAYING", stopped=False, secs_remaining=65)
        self.node = Mock()
        self.dashboard = Dashboard(self.node)

    def snapshot(self):
        self.dashboard.update(self.state, 12_000_000_000, None, True)
        message = self.node.create_publisher.return_value.publish.call_args.args[0]
        return json.loads(message.data)

    def test_snapshot_contains_match_and_history_for_reopened_window(self):
        self.dashboard.record("Startablauf: STATE_SET → STATE_PLAYING")
        self.dashboard.record("Ballkontakt: Team 1", 12_000_000_000)
        data = self.snapshot()
        self.assertEqual(data["match"]["secs_remaining"], 65)
        self.assertEqual(
            [event["message"] for event in data["events"]],
            ["Ballkontakt: Team 1", "Startablauf: STATE_SET → STATE_PLAYING"],
        )
        self.assertEqual(self.snapshot()["events"], data["events"])

    def test_display_uses_received_time_without_extrapolation(self):
        data = self.snapshot()
        _, _, phase, clock, details, _ = format_snapshot(data)
        self.assertEqual(phase, "PLAYING")
        self.assertEqual(clock, "01:05")
        self.assertIn("Unterbrochen: Nein", details[0])
        self.assertIn("12.00 s", details[2])
        self.assertEqual(format_snapshot(data)[3], clock)

    def test_empty_history_and_missing_robot_are_displayable(self):
        self.dashboard.update(self.state, None, None, None)
        message = self.node.create_publisher.return_value.publish.call_args.args[0]
        *_, details, events = format_snapshot(json.loads(message.data))
        self.assertIn("Warte auf Antwort", details[1])
        self.assertIn("Noch keine Daten", details[2])
        self.assertEqual(events, [])

    def test_invalid_snapshot_keeps_last_good_state(self):
        subscriber = Mock()
        subscriber.snapshot = self.snapshot()
        subscriber.received_at = 1.0
        previous = subscriber.snapshot
        DashboardSubscriber._receive(subscriber, String(data='{"match": {}}'))
        self.assertIs(subscriber.snapshot, previous)
        self.assertEqual(subscriber.received_at, 1.0)
        subscriber.get_logger.return_value.warning.assert_called_once()

    def test_invalid_published_timestamp_is_rejected(self):
        data = self.snapshot()
        data["published_at"] = float("nan")
        with self.assertRaises(ValueError):
            format_snapshot(data)
