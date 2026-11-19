"""Defer robot placements until observed motion has stopped and placement is acknowledged."""

import math
from concurrent.futures import Future
from dataclasses import dataclass, field

from bitbots_auto_referee.core.teleport import TeleportResult
from bitbots_auto_referee.rules.motion import ANGULAR_SPEED_LIMIT, JOINT_SPEED_LIMIT, LINEAR_SPEED_LIMIT


@dataclass
class RobotPlacement:
    pose: tuple[float, float, float]
    completion: Future[TeleportResult] = field(default_factory=Future)
    command: Future[TeleportResult] | None = None
    failed: bool = False


class RobotPlacements:
    def __init__(self, commands, event):
        self.commands = commands
        self.event = event
        self.pending: dict[int, RobotPlacement] = {}

    def request(self, robot, x, y, yaw):
        if robot not in self.pending:
            self.pending[robot] = RobotPlacement((x, y, yaw))
        return self.pending[robot].completion

    def clear(self):
        pending, self.pending = self.pending, {}
        for placement in pending.values():
            if not placement.completion.done():
                placement.completion.cancel()

    def advance(self, observation):
        for robot, placement in list(self.pending.items()):
            if placement.failed:
                continue
            try:
                if placement.command is None:
                    motion = observation.robot_motion.get(robot)
                    if (
                        robot not in observation.robot_positions
                        or robot in observation.teleported_robots
                        or motion is None
                        or not all(math.isfinite(value) for value in vars(motion).values())
                        or abs(motion.linear_speed) > LINEAR_SPEED_LIMIT
                        or abs(motion.angular_speed) > ANGULAR_SPEED_LIMIT
                        or abs(motion.body_joint_speed) > JOINT_SPEED_LIMIT
                        or abs(motion.head_joint_speed) > JOINT_SPEED_LIMIT
                    ):
                        continue
                    if self.commands is None:
                        raise RuntimeError("No simulator command adapter configured")
                    placement.command = self.commands.teleport_robot(robot, *placement.pose)
                if not placement.command.done():
                    continue
                result = placement.command.result()
                if not result.success:
                    raise RuntimeError(result.message)
                if observation.step_number < result.applied_step:
                    continue
                del self.pending[robot]
                placement.completion.set_result(result)
            except Exception as error:
                placement.failed = True
                placement.completion.set_result(TeleportResult(False, str(error)))
                self.event(f"Roboterplatzierung {robot} fehlgeschlagen; Spiel bleibt angehalten: {error}")
