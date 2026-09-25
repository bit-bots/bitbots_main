"""Shared launch defaults and validated startup configuration."""

import ipaddress
import json
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ParameterSpec:
    default: str | int | float | bool
    description: str
    choices: tuple[str, ...] = ()


COLORS = ("blue", "red", "yellow", "black", "white", "green", "orange", "purple", "brown", "gray")
LEAGUES = ("small", "middle", "large")
STATES = ("initial", "ready", "set", "playing", "finished")
PLAYERS_PER_TEAM = {
    "small": {"foundation": 4, "advanced": 7},
    "middle": {"foundation": 3, "advanced": 5},
    "large": {"foundation": 2, "advanced": 3},
}

PARAMETERS = {
    "double_touch_min_force": ParameterSpec(
        2.0, "Minimum peak robot-ball contact force in newtons for a significant touch."
    ),
    "double_touch_min_impulse": ParameterSpec(
        0.02, "Minimum robot-ball contact impulse in newton-seconds for a significant touch."
    ),
    "double_touch_release_time": ParameterSpec(
        0.05, "Minimum separation in simulation seconds before a new touch can count."
    ),
    "pushing_force_threshold": ParameterSpec(20.0, "Minimum contact force in newtons for a destabilizing single push."),
    "pushing_sustained_force_threshold": ParameterSpec(2.0, "Minimum contact force in newtons for sustained pushing."),
    "pushing_approach_speed": ParameterSpec(0.03, "Minimum approach speed in m/s for assigning a pushing actor."),
    "pushing_tilt_drop": ParameterSpec(0.15, "Minimum decrease in torso upright projection after contact."),
    "pushing_angular_speed": ParameterSpec(1.0, "Minimum increase in victim angular speed in rad/s after contact."),
    "pushing_effect_window": ParameterSpec(0.5, "Seconds after contact during which destabilization is attributed."),
    "pushing_ball_center_tolerance": ParameterSpec(
        0.25, "Maximum ball offset from the pair midpoint in metres for a legal duel."
    ),
    "pushing_ball_reach": ParameterSpec(0.7, "Maximum robot-ball distance in metres for a legal duel."),
    "leagueSize": ParameterSpec(
        "small", "Competition size; together with lineup_mode sets the per-team limit.", LEAGUES
    ),
    "lineup_mode": ParameterSpec(
        "foundation",
        "Together with leagueSize sets the per-team limit; fewer or no connected robots are allowed.",
        ("foundation", "advanced"),
    ),
    "home_team_id": ParameterSpec(1, "Home team number; must match the home receiver's team_id."),
    "away_team_id": ParameterSpec(2, "Away team number; must differ from home_team_id."),
    "robot_player_mapping": ParameterSpec(
        "{}", "Optional JSON robot-index to player-number overrides; otherwise ordered within each team."
    ),
    "center_circle_radius": ParameterSpec(0.75, "Center circle radius to the marking center in metres."),
    "penalty_area_width": ParameterSpec(4.0, "Penalty area width in metres."),
    "penalty_area_length": ParameterSpec(2.0, "Penalty area depth in metres."),
    "robot_team_mapping": ParameterSpec('{"0":"home"}', "JSON object mapping simulator robot indices to home or away."),
    "home_color": ParameterSpec("blue", "Home field-player jersey color.", COLORS),
    "away_color": ParameterSpec("red", "Away field-player jersey color.", COLORS),
    "home_goalkeeper_color": ParameterSpec("blue", "Home goalkeeper jersey color.", COLORS),
    "away_goalkeeper_color": ParameterSpec("red", "Away goalkeeper jersey color.", COLORS),
    "target_host": ParameterSpec("127.0.0.1", "Unicast IPv4 address of the robot receiver."),
    "target_port": ParameterSpec(3838, "Must match the receiver's listen_port."),
    "bind_host": ParameterSpec("127.0.0.1", "Local IPv4 interface for sending and receiving UDP packets."),
    "return_port": ParameterSpec(3939, "Must match the receiver's answer_port."),
    "send_rate": ParameterSpec(2.0, "Packet frequency in wall-clock hertz, including while simulation is paused."),
    "response_timeout": ParameterSpec(5.0, "Wall-clock seconds without a reply before reporting a lost connection."),
    "field_length": ParameterSpec(9.0, "Distance between goal-line centers in metres."),
    "field_width": ParameterSpec(6.0, "Distance between touchline centers in metres."),
    "line_width": ParameterSpec(0.05, "Field marking width in metres."),
    "goal_width": ParameterSpec(1.75, "Clear goal opening width in metres."),
    "goal_height": ParameterSpec(1.2, "Clear goal opening height in metres."),
    "goal_area_length": ParameterSpec(1.0, "Goal area depth in metres."),
    "goal_area_width": ParameterSpec(3.0, "Goal area width in metres."),
    "ball_radius": ParameterSpec(0.07, "Simulated ball radius in metres."),
    "home_defends_negative_x": ParameterSpec(
        False, "Home defends negative X in the first half; sides swap at halftime."
    ),
    "ui_enabled": ParameterSpec(True, "Launch the native read-only AutoRef window."),
    "use_sim_time": ParameterSpec(True, "Use the simulator's clock for the opening sequence and referee decisions."),
}


@dataclass(frozen=True)
class RefereeConfig:
    double_touch_min_force: float
    double_touch_min_impulse: float
    double_touch_release_time: float
    pushing_force_threshold: float
    pushing_sustained_force_threshold: float
    pushing_approach_speed: float
    pushing_tilt_drop: float
    pushing_angular_speed: float
    pushing_effect_window: float
    pushing_ball_center_tolerance: float
    pushing_ball_reach: float
    league_size: str
    lineup_mode: str
    home_team_id: int
    away_team_id: int
    robot_player_mapping: str
    center_circle_radius: float
    penalty_area_width: float
    penalty_area_length: float
    robot_team_mapping: str
    home_color: str
    away_color: str
    home_goalkeeper_color: str
    away_goalkeeper_color: str
    target_host: str
    target_port: int
    bind_host: str
    return_port: int
    send_rate: float
    response_timeout: float
    ui_enabled: bool
    field_length: float
    field_width: float
    line_width: float
    goal_width: float
    goal_height: float
    goal_area_length: float
    goal_area_width: float
    ball_radius: float
    home_defends_negative_x: bool

    @property
    def robot_teams(self) -> dict[int, int]:
        """Resolve explicit simulator assignments against the configured team identities."""
        mapping = json.loads(self.robot_team_mapping)
        if not isinstance(mapping, dict):
            raise ValueError("robot_team_mapping must be a JSON object")
        teams = {"home": self.home_team_id, "away": self.away_team_id}
        result = {}
        for index, side in mapping.items():
            if not index.isascii() or not index.isdecimal() or str(int(index)) != index:
                raise ValueError("Robot indices must be canonical nonnegative integers")
            if int(index) > 0xFFFFFFFF:
                raise ValueError("Robot index exceeds the simulation message range")
            if not isinstance(side, str) or side not in teams:
                raise ValueError("Robot team assignments must be home or away")
            result[int(index)] = teams[side]
        for team_id in teams.values():
            if sum(team == team_id for team in result.values()) > self.players_per_team:
                raise ValueError("Robot team mapping exceeds the per-team player limit")
        return result

    @property
    def robot_players(self) -> dict[int, int]:
        teams = self.robot_teams
        result = {}
        for team in (self.home_team_id, self.away_team_id):
            indices = sorted(index for index, team_id in teams.items() if team_id == team)
            result.update({index: number for number, index in enumerate(indices, 1)})
        overrides = json.loads(self.robot_player_mapping)
        if not isinstance(overrides, dict):
            raise ValueError("robot_player_mapping must be a JSON object")
        for index, number in overrides.items():
            if not index.isascii() or not index.isdecimal() or str(int(index)) != index or int(index) not in teams:
                raise ValueError("Player mapping requires a configured robot index")
            if type(number) is not int or not 1 <= number <= self.players_per_team:
                raise ValueError("Player number must fit the team roster")
            result[int(index)] = number
        for team in (self.home_team_id, self.away_team_id):
            numbers = [number for index, number in result.items() if teams[index] == team]
            if len(numbers) != len(set(numbers)):
                raise ValueError("Player numbers must be unique within each team")
        return result

    @property
    def players_per_team(self) -> int:
        """Derive the per-team upper limit, independently of connected robots."""
        return PLAYERS_PER_TEAM[self.league_size][self.lineup_mode]

    @classmethod
    def from_parameters(cls, values: dict) -> "RefereeConfig":
        unknown = values.keys() - PARAMETERS.keys()
        if unknown:
            raise ValueError(f"Unknown referee parameters: {', '.join(sorted(unknown))}")
        for name, spec in PARAMETERS.items():
            value = values[name]
            if type(value) is not type(spec.default):
                raise ValueError(f"{name} must have type {type(spec.default).__name__}")
            if spec.choices and value not in spec.choices:
                raise ValueError(f"{name} must be one of {', '.join(spec.choices)}")

        for name in ("home_team_id", "away_team_id"):
            if not 1 <= values[name] < 255:
                raise ValueError(f"{name} must be a team number below the reserved no-team value")
        if values["home_team_id"] == values["away_team_id"]:
            raise ValueError("Home and away team IDs must be different")
        if values["home_color"] == values["away_color"]:
            raise ValueError("Home and away field-player colors must be different")
        for name in ("target_port", "return_port"):
            if not 1 <= values[name] <= 65535:
                raise ValueError(f"{name} must be a valid port")
        if values["target_port"] == values["return_port"]:
            raise ValueError("Receiver and return ports must be different")
        for name in ("target_host", "bind_host"):
            address = ipaddress.IPv4Address(values[name])
            if address.is_multicast or int(address) == 0xFFFFFFFF:
                raise ValueError(f"{name} must be a unicast IPv4 address")
            if name == "target_host" and address.is_unspecified:
                raise ValueError("target_host cannot be the wildcard address")
        for name in ("send_rate", "response_timeout"):
            if not math.isfinite(values[name]) or values[name] <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if values["send_rate"] > 100:
            raise ValueError("send_rate is too high for a GameController heartbeat")

        for name in (
            "field_length",
            "field_width",
            "line_width",
            "goal_width",
            "goal_height",
            "goal_area_length",
            "goal_area_width",
            "ball_radius",
            "penalty_area_length",
            "penalty_area_width",
            "center_circle_radius",
        ):
            if not math.isfinite(values[name]) or values[name] <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not values["goal_width"] <= values["goal_area_width"] <= values["field_width"]:
            raise ValueError("Goal and goal area widths must fit inside the field")
        if values["goal_area_length"] >= values["field_length"] / 2:
            raise ValueError("Goal area must fit inside its half")
        if 2 * values["ball_radius"] >= min(values["goal_width"], values["goal_height"]):
            raise ValueError("Ball must fit through the goal opening")
        if values["line_width"] >= min(values["field_length"], values["field_width"]):
            raise ValueError("Line width must be smaller than the field dimensions")

        for name in (name for name in PARAMETERS if name.startswith(("pushing_", "double_touch_"))):
            if not math.isfinite(values[name]) or values[name] <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if values["pushing_sustained_force_threshold"] > values["pushing_force_threshold"]:
            raise ValueError("Sustained pushing force must not exceed the single-push threshold")
        if values["pushing_tilt_drop"] > 2:
            raise ValueError("Pushing tilt drop exceeds the upright projection range")
        fields = {name: value for name, value in values.items() if name not in ("leagueSize", "use_sim_time")}
        config = cls(league_size=values["leagueSize"], **fields)
        if not values["goal_area_length"] <= values["penalty_area_length"] < values["field_length"] / 2:
            raise ValueError("Penalty area must contain the goal area and fit its half")
        if not values["goal_area_width"] <= values["penalty_area_width"] <= values["field_width"]:
            raise ValueError("Penalty area width must contain the goal area and fit the field")
        if 2 * values["center_circle_radius"] >= min(values["field_length"], values["field_width"]):
            raise ValueError("Center circle must fit inside the field")
        _ = config.robot_players
        return config
