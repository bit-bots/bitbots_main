"""End READY early after a complete legal lineup has remained stationary."""

import math

from bitbots_auto_referee.core.observations import SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.rules.motion import ANGULAR_SPEED_LIMIT, JOINT_SPEED_LIMIT, LINEAR_SPEED_LIMIT
from bitbots_auto_referee.rules.startup import NANOSECONDS_PER_SECOND

READY_STILL_NS = 5 * NANOSECONDS_PER_SECOND
POSITION_TOLERANCE = 0.03
YAW_TOLERANCE = 0.1


class ReadyRules:
    def __init__(self, positions):
        self.positions = positions
        self._since = None
        self._anchors = {}
        self._context = None
        self._previous = None

    def check(self, state: MatchState, observation: SimulationObservation) -> bool:
        previous = self._previous
        self._previous = observation
        context = (state.state, state.stopped, state.kicking_team, state.first_half)
        reset = previous is not None and (
            observation.time_ns < previous.time_ns or observation.step_number < previous.step_number
        )
        if reset or context != self._context:
            self._since = None
            self._anchors.clear()
        self._context = context
        robots = self.positions.legal_set_robots(state, observation) if state.state == "STATE_READY" else None
        poses = {}
        if robots is None or state.stopped or robots.intersection(observation.teleported_robots):
            self._since = None
            self._anchors.clear()
            return False
        for robot in robots:
            motion = observation.robot_motion.get(robot)
            yaw = observation.robot_yaws.get(robot)
            position = observation.robot_positions[robot]
            if (
                motion is None
                or yaw is None
                or not all(
                    math.isfinite(value)
                    for value in (*position, yaw, motion.linear_speed, motion.angular_speed, motion.body_joint_speed)
                )
                or abs(motion.linear_speed) > LINEAR_SPEED_LIMIT
                or abs(motion.angular_speed) > ANGULAR_SPEED_LIMIT
                or abs(motion.body_joint_speed) > JOINT_SPEED_LIMIT
            ):
                self._since = None
                self._anchors.clear()
                return False
            poses[robot] = (position[:2], yaw)
        moved = poses.keys() != self._anchors.keys() or any(
            math.dist(position, self._anchors[robot][0]) > POSITION_TOLERANCE
            or abs(math.remainder(yaw - self._anchors[robot][1], math.tau)) > YAW_TOLERANCE
            for robot, (position, yaw) in poses.items()
            if robot in self._anchors
        )
        if self._since is None or moved:
            self._since = observation.time_ns
            self._anchors = poses
        return observation.time_ns - self._since >= READY_STILL_NS
