"""Simulation-time stuck play, recovery failures and conservative ball holding."""

import math
from dataclasses import replace

from bitbots_auto_referee.core.observations import SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.rules.startup import NANOSECONDS_PER_SECOND, STARTUP_PHASES

GLOBAL_STUCK_NS = 30 * NANOSECONDS_PER_SECOND
LOCAL_STUCK_NS = 10 * NANOSECONDS_PER_SECOND
INCAPABLE_NS = 20 * NANOSECONDS_PER_SECOND
HOLDING_NS = 5 * NANOSECONDS_PER_SECOND
RECOVERED_NS = NANOSECONDS_PER_SECOND
BALL_MOVEMENT = 0.03
PLAYABLE_DISTANCE = 0.6
MAX_RECOVERY_FAILURES = 3


class GameFlowRules:
    def __init__(self, penalties, field, teleport_robot, teleport_ball, event):
        self.penalties = penalties
        self.field = field
        self.teleport_robot = teleport_robot
        self.teleport_ball = teleport_ball
        self.event = event
        self.initial_poses = {}
        self._previous = None
        self._anchor = None
        self._free_since = None
        self._local_since = {}
        self._fallen = {}
        self._holding = {}
        self._placements = None
        self._restart_at = None
        self._restart_failed = False

    def _eligible(self, state, robot):
        team_id = self.penalties.robot_teams.get(robot)
        number = self.penalties.robot_players.get(robot)
        return any(
            team.team_number == team_id and number is not None and 1 <= number <= len(team.players)
            and team.players[number - 1].penalty == "PENALTY_NONE"
            for team in state.teams
        )

    def _reset_play(self):
        self._anchor = None
        self._free_since = None
        self._local_since.clear()
        self._holding.clear()

    def _playable(self, robot, observation):
        motion = observation.robot_motion.get(robot)
        yaw = observation.robot_yaws.get(robot)
        if motion is None or yaw is None or not math.isfinite(yaw):
            return False
        if motion.upright < 0.8 or motion.relative_height < 0.7:
            return False
        x, y, _ = observation.robot_positions[robot]
        bx, by, _ = observation.ball_position
        dx, dy = bx - x, by - y
        radius = math.hypot(dx, dy)
        facing = dx * math.cos(yaw) + dy * math.sin(yaw)
        return radius <= PLAYABLE_DISTANCE and facing >= radius * 0.5

    def _check_incapable(self, state, observation):
        now = observation.time_ns
        for robot in list(self._fallen):
            if robot not in observation.robot_positions:
                del self._fallen[robot]
        for robot in observation.robot_positions:
            motion = observation.robot_motion.get(robot)
            if (
                not self._eligible(state, robot) or robot in observation.teleported_robots
                or motion is None or not all(math.isfinite(value) for value in vars(motion).values())
            ):
                self._fallen.pop(robot, None)
                continue
            down = motion.upright < 0.5 or motion.relative_height < 0.5
            upright = motion.upright > 0.85 and motion.relative_height > 0.8
            recovery = self._fallen.get(robot)
            if recovery is None:
                if not down:
                    continue
                recovery = {"since": now, "attempt": False, "failures": 0, "upright_since": None}
                self._fallen[robot] = recovery
            if upright:
                if recovery["upright_since"] is None:
                    recovery["upright_since"] = now
                if now - recovery["upright_since"] >= RECOVERED_NS:
                    del self._fallen[robot]
                    continue
            else:
                recovery["upright_since"] = None
            # Hysteresis: a meaningful partial rise followed by another fall is one failed attempt.
            if motion.upright > 0.7 and motion.relative_height > 0.65:
                recovery["attempt"] = True
            elif down and recovery["attempt"]:
                recovery["attempt"] = False
                recovery["failures"] += 1
            if now - recovery["since"] >= INCAPABLE_NS or recovery["failures"] >= MAX_RECOVERY_FAILURES:
                state = self.penalties.penalize(state, observation, robot, "PENALTY_INCAPABLE_ROBOT")
                self._fallen.pop(robot, None)
        return state

    def _begin_restart(self, state, observation):
        self._placements = []
        self._restart_failed = False
        self._reset_play()
        self._fallen.clear()
        self.event("Global Gamestuck: Ball zur Mitte, Roboter zur Ausgangsaufstellung; neutraler Neustart.")
        try:
            missing = observation.robot_positions.keys() - self.initial_poses.keys()
            if missing:
                raise RuntimeError(f"Ausgangsposition oder Blickrichtung fehlt für Roboter {sorted(missing)}")
            for robot in sorted(observation.robot_positions):
                self._placements.append(self.teleport_robot(robot, *self.initial_poses[robot]))
            self._placements.append(self.teleport_ball(0.0, 0.0, 0.0))
        except Exception as error:
            self._restart_failed = True
            self.event(f"Globaler Neustart angehalten: {error}")
        return replace(state, stopped=True, kicking_team=255, set_play="SET_PLAY_NONE", secondary_time=0)

    def _advance_restart(self, state, observation):
        if self._restart_failed:
            return replace(state, stopped=True)
        if self._placements is not None:
            try:
                applied = []
                for future in self._placements:
                    if future.done():
                        outcome = future.result()
                        if not outcome.success:
                            raise RuntimeError(outcome.message)
                        applied.append(outcome.applied_step)
                if len(applied) != len(self._placements) or any(step > observation.step_number for step in applied):
                    return replace(state, stopped=True)
            except Exception as error:
                self._restart_failed = True
                self.event(f"Globaler Neustart angehalten: {error}")
                return replace(state, stopped=True)
            self._placements = None
            self._restart_at = observation.time_ns
            self.event("Globale Neuaufstellung abgeschlossen: READY ohne Anstoßteam.")
        elapsed = observation.time_ns - self._restart_at
        boundary = 0
        for phase, seconds in STARTUP_PHASES:
            if phase == "STATE_INITIAL":
                continue
            boundary += seconds * NANOSECONDS_PER_SECOND
            if elapsed < boundary:
                remaining = (boundary - elapsed + NANOSECONDS_PER_SECOND - 1) // NANOSECONDS_PER_SECOND
                return replace(state, state=phase, stopped=False, secondary_time=remaining, kicking_team=255)
        self._restart_at = None
        self._reset_play()
        self.event("Globaler Neustart: PLAYING ohne Anstoßteam.")
        return replace(state, state="STATE_PLAYING", stopped=False, secondary_time=0, kicking_team=255)

    def check(self, state: MatchState, observation: SimulationObservation) -> MatchState:
        previous = self._previous
        reset = previous is not None and (
            observation.time_ns < previous.time_ns or observation.step_number < previous.step_number
        )
        restart_on_reset = reset and (self._placements is not None or self._restart_at is not None)
        if reset:
            self._reset_play()
            self._fallen.clear()
            self.initial_poses.clear()
            self._placements = None
            self._restart_failed = False
            self._restart_at = None
        self._previous = observation
        for robot, (x, y, _) in observation.robot_positions.items():
            yaw = observation.robot_yaws.get(robot)
            if yaw is not None and all(math.isfinite(value) for value in (x, y, yaw)):
                # Keep existing sideline spawns; project in-field starts to their nearest touchline.
                side_y = math.copysign(max(abs(y), self.field.field_width / 2), y)
                self.initial_poses.setdefault(robot, (x, side_y, yaw))
        if restart_on_reset:
            return self._begin_restart(state, observation)
        if self._placements is not None or self._restart_at is not None:
            state = self._advance_restart(state, observation)
            return self._check_incapable(state, observation) if not state.stopped else state
        state = self._check_incapable(state, observation)
        free = state.state == "STATE_PLAYING" and not state.stopped and state.set_play == "SET_PLAY_NONE"
        ball = observation.ball_position
        if not free or ball is None or not all(math.isfinite(value) for value in ball) or observation.ball_teleported:
            self._reset_play()
            return state
        if self._anchor is None or math.dist(ball, self._anchor) > BALL_MOVEMENT:
            self._anchor = ball
            self._free_since = observation.time_ns
            self._local_since.clear()
            self._holding.clear()
        now = observation.time_ns
        for robot in list(self._holding):
            if robot not in observation.robot_positions:
                del self._holding[robot]
        for robot in observation.robot_positions:
            motion = observation.robot_motion.get(robot)
            holding = (
                self._eligible(state, robot) and robot not in observation.teleported_robots
                and motion is not None and motion.upright > 0.85 and motion.relative_height > 0.8
                and observation.ball_blockage.get(robot, 0.0) >= 0.875
            )
            if holding:
                since = self._holding.setdefault(robot, now)
                if now - since >= HOLDING_NS:
                    state = self.penalties.penalize(state, observation, robot, "PENALTY_BALL_HOLDING")
                    self._holding.pop(robot, None)
            else:
                self._holding.pop(robot, None)
        near = {
            robot for robot in observation.robot_positions
            if self._eligible(state, robot) and robot not in observation.teleported_robots
            and self._playable(robot, observation)
        }
        self._local_since = {robot: since for robot, since in self._local_since.items() if robot in near}
        for robot in near:
            self._local_since.setdefault(robot, now)
        overdue = [robot for robot in near if now - self._local_since[robot] >= LOCAL_STUCK_NS]
        if overdue:
            robot = min(near, key=lambda index: (math.dist(observation.robot_positions[index][:2], ball[:2]), index))
            state = self.penalties.penalize(state, observation, robot, "PENALTY_LOCAL_GAME_STUCK")
            self._local_since.clear()
        if now - self._free_since >= GLOBAL_STUCK_NS:
            return self._begin_restart(state, observation)
        return state
