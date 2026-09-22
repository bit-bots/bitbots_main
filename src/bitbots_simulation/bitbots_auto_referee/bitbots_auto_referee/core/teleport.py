"""Asynchronous simulation commands available to the rule checker."""

from concurrent.futures import Future
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class TeleportResult:
    success: bool
    message: str
    applied_step: int = 0


class TeleportCommands(Protocol):
    def teleport_robot(self, robot_id: int, x: float, y: float, yaw: float) -> Future[TeleportResult]: ...

    def teleport_ball(self, x: float, y: float, yaw: float) -> Future[TeleportResult]: ...
