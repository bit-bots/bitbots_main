"""Nonblocking client for acknowledged teleports in the running simulator."""

import math
from collections.abc import Callable
from concurrent.futures import Future

from bitbots_msgs.srv import Teleport
from rclpy.node import Node

from bitbots_auto_referee.core.teleport import TeleportResult


class SimulationCommands:
    def __init__(self, node: Node, event_callback: Callable[[str], None]):
        self._client = node.create_client(Teleport, "/simulation/teleport")
        self._event_callback = event_callback

    def teleport_robot(self, robot_id: int, x: float, y: float, yaw: float) -> Future[TeleportResult]:
        if type(robot_id) is not int or not 0 <= robot_id <= 0xFFFFFFFF:
            raise ValueError("robot_id must be a valid simulator index")
        return self._request(Teleport.Request.ROBOT, robot_id, x, y, yaw)

    def teleport_ball(self, x: float, y: float, yaw: float) -> Future[TeleportResult]:
        return self._request(Teleport.Request.BALL, 0, x, y, yaw)

    def _request(self, target: int, robot_id: int, x: float, y: float, yaw: float) -> Future[TeleportResult]:
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("Teleport coordinates and yaw must be finite")
        result: Future[TeleportResult] = Future()
        label = "Ball" if target == Teleport.Request.BALL else f"Roboter {robot_id}"
        if not self._client.service_is_ready():
            message = "Simulator-Teleportdienst ist nicht erreichbar"
            result.set_result(TeleportResult(False, message))
            self._event_callback(f"{label}: {message}")
            return result
        request = Teleport.Request(target_type=target, robot_index=robot_id, x=float(x), y=float(y), yaw=float(yaw))

        def completed(response_future):
            try:
                response = response_future.result()
                outcome = TeleportResult(response.success, response.message, response.applied_step)
            except Exception as error:
                outcome = TeleportResult(False, str(error))
            if not result.cancelled():
                result.set_result(outcome)
            self._event_callback(f"{label}: Teleport {'ausgeführt' if outcome.success else 'fehlgeschlagen'} – {outcome.message}")

        try:
            self._client.call_async(request).add_done_callback(completed)
        except Exception as error:
            result.set_result(TeleportResult(False, str(error)))
        return result
