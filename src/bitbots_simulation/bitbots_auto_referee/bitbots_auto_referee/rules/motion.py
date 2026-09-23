"""Simulation-time motion penalties with per-robot grace and recovery tracking."""

import math
from dataclasses import replace

from bitbots_auto_referee.core.observations import SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.rules.startup import NANOSECONDS_PER_SECOND

GRACE_NS = 1_000_000_000
VIOLATION_NS = 250_000_000
LINEAR_SPEED_LIMIT = 0.03
ANGULAR_SPEED_LIMIT = 0.2
JOINT_SPEED_LIMIT = 0.3
UPRIGHT_LIMIT = 0.7
HEIGHT_LIMIT = 0.7
PENALTY_SECONDS = {"PENALTY_MOTION_IN_SET": 15, "PENALTY_MOTION_IN_STOP": 45}


class MotionRules:
    def __init__(self, robot_teams, robot_players, field, teleport_robot, event):
        self.robot_teams = robot_teams
        self.robot_players = robot_players
        self.field = field
        self.teleport_robot = teleport_robot
        self.event = event
        self._mode = None
        self._last_time = None
        self._last_step = None
        self._grace = {}
        self._moving_since = {}
        self._penalties = {}

    def check(self, state: MatchState, observation: SimulationObservation) -> MatchState:
        now = observation.time_ns
        reset = self._last_time is not None and (
            now < self._last_time or observation.step_number < self._last_step
        )
        self._last_time, self._last_step = now, observation.step_number
        mode = "PENALTY_MOTION_IN_STOP" if state.stopped else (
            "PENALTY_MOTION_IN_SET" if state.state == "STATE_SET" else None
        )
        if reset or mode != self._mode:
            self._grace.clear()
            self._moving_since.clear()
        self._mode = mode
        teams = list(state.teams)
        for team_index, team in enumerate(teams):
            players = list(team.players)
            for player_index, player in enumerate(players):
                key = (team.team_number, player_index + 1)
                owned = self._penalties.get(key)
                if owned is None:
                    continue
                penalty, deadline = owned
                if player.penalty != penalty:
                    del self._penalties[key]
                    continue
                remaining = 0 if reset else max(0, (deadline - now + NANOSECONDS_PER_SECOND - 1) // NANOSECONDS_PER_SECOND)
                players[player_index] = replace(
                    player, penalty=penalty if remaining else "PENALTY_NONE", secs_till_unpenalized=remaining
                )
                if not remaining:
                    del self._penalties[key]
                    self.event(f"Strafe aufgehoben: Team {team.team_number}, Spieler {player_index + 1}")
                    for robot, number in self.robot_players.items():
                        if self.robot_teams[robot] == team.team_number and number == player_index + 1:
                            self._grace[robot] = now + GRACE_NS
                            self._moving_since.pop(robot, None)
            teams[team_index] = replace(team, players=tuple(players))
        for robot in list(self._grace):
            if robot not in observation.robot_positions:
                self._grace.pop(robot, None)
                self._moving_since.pop(robot, None)
        for robot in observation.robot_positions:
            self._grace.setdefault(robot, now + GRACE_NS)
            if robot in observation.teleported_robots:
                self._grace[robot] = now + GRACE_NS
                self._moving_since.pop(robot, None)
            motion = observation.robot_motion.get(robot)
            if mode is None or motion is None:
                self._moving_since.pop(robot, None)
                continue
            if not all(math.isfinite(value) for value in vars(motion).values()):
                self._moving_since.pop(robot, None)
                continue
            if mode == "PENALTY_MOTION_IN_SET" and (
                motion.upright < UPRIGHT_LIMIT or motion.relative_height < HEIGHT_LIMIT
            ):
                self._grace[robot] = now + GRACE_NS
                self._moving_since.pop(robot, None)
                continue
            moving = (
                motion.linear_speed > LINEAR_SPEED_LIMIT
                or motion.angular_speed > ANGULAR_SPEED_LIMIT
                or motion.body_joint_speed > JOINT_SPEED_LIMIT
                or (mode == "PENALTY_MOTION_IN_STOP" and motion.head_joint_speed > JOINT_SPEED_LIMIT)
            )
            if now < self._grace[robot] or not moving:
                self._moving_since.pop(robot, None)
                continue
            started = self._moving_since.setdefault(robot, now)
            if now - started < VIOLATION_NS:
                continue
            team_id = self.robot_teams.get(robot)
            number = self.robot_players.get(robot)
            team_index = next((i for i, team in enumerate(teams) if team.team_number == team_id), None)
            if team_index is None or number is None or not 1 <= number <= len(teams[team_index].players):
                continue
            team = teams[team_index]
            players = list(team.players)
            if players[number - 1].penalty != "PENALTY_NONE":
                continue
            duration = PENALTY_SECONDS[mode]
            players[number - 1] = replace(players[number - 1], penalty=mode, secs_till_unpenalized=duration)
            teams[team_index] = replace(team, players=tuple(players))
            self._penalties[(team_id, number)] = (mode, now + duration * NANOSECONDS_PER_SECOND)
            self.event(f"{mode}: Team {team_id}, Spieler {number}, Roboter {robot}")
            if mode == "PENALTY_MOTION_IN_STOP":
                self._relocate(robot, team_id, state, observation)
        return replace(state, teams=tuple(teams)) if tuple(teams) != state.teams else state

    def _relocate(self, robot, team_id, state, observation):
        home_negative = self.field.home_defends_negative_x == state.first_half
        own_negative = home_negative if team_id == state.teams[0].team_number else not home_negative
        x = (-1 if own_negative else 1) * (self.field.field_length / 2 - self.field.penalty_area_length)
        side = 1 if observation.robot_positions[robot][1] >= 0 else -1
        y = side * self.field.field_width / 2
        yaw = -side * math.pi / 2

        def completed(future):
            try:
                result = future.result()
                if not result.success:
                    self.event(f"Strafplatzierung Roboter {robot} fehlgeschlagen: {result.message}")
            except Exception as error:
                self.event(f"Strafplatzierung Roboter {robot} fehlgeschlagen: {error}")

        try:
            self.teleport_robot(robot, x, y, yaw).add_done_callback(completed)
        except Exception as error:
            self.event(f"Strafplatzierung Roboter {robot} fehlgeschlagen: {error}")
