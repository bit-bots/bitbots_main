"""Publish read-only dashboard snapshots without importing a graphical backend."""

import json
import time
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from bitbots_auto_referee.core.state import MatchState

DASHBOARD_TOPIC = "dashboard"


def dashboard_qos() -> QoSProfile:
    """Retain the newest complete snapshot for a newly opened application."""
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Dashboard:
    def __init__(self, node: Node):
        self._node = node
        self._publisher = node.create_publisher(String, DASHBOARD_TOPIC, dashboard_qos())
        self._events: deque[dict] = deque(maxlen=200)

    def record(self, message: str, simulation_time_ns: int | None = None) -> None:
        self._events.appendleft(
            {
                "time": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                "simulation_time_ns": simulation_time_ns,
                "message": message,
            }
        )

    def update(
        self,
        state: MatchState,
        simulation_time_ns: int | None,
        last_touch_team_id: int | None,
        connected: bool | None,
    ) -> None:
        snapshot = {
            "match": asdict(state),
            "simulation_time_ns": simulation_time_ns,
            "last_touch_team_id": last_touch_team_id,
            "robot_connected": connected,
            "events": list(self._events),
            "published_at": time.time(),
        }
        self._publisher.publish(String(data=json.dumps(snapshot, allow_nan=False)))

    def close(self) -> None:
        self._node.destroy_publisher(self._publisher)
