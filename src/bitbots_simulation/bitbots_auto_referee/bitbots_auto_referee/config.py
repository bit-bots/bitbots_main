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
    "leagueSize": ParameterSpec("small", "Competition size; together with lineup_mode sets the per-team limit.", LEAGUES),
    "lineup_mode": ParameterSpec(
        "foundation",
        "Together with leagueSize sets the per-team limit; fewer or no connected robots are allowed.",
        ("foundation", "advanced"),
    ),
    "home_team_id": ParameterSpec(1, "Home team number; must match the home receiver's team_id."),
    "away_team_id": ParameterSpec(2, "Away team number; must differ from home_team_id."),
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
    "home_defends_negative_x": ParameterSpec(False, "Home defends negative X in the first half; sides swap at halftime."),
    "ui_enabled": ParameterSpec(True, "Launch the native read-only AutoRef window."),
    "use_sim_time": ParameterSpec(True, "Use the simulator's clock for the opening sequence and referee decisions."),
}


@dataclass(frozen=True)
class RefereeConfig:
    league_size: str
    lineup_mode: str
    home_team_id: int
    away_team_id: int
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

        for name in ("field_length", "field_width", "line_width", "goal_width", "goal_height",
                     "goal_area_length", "goal_area_width", "ball_radius"):
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

        fields = {name: value for name, value in values.items() if name not in ("leagueSize", "use_sim_time")}
        config = cls(league_size=values["leagueSize"], **fields)
        _ = config.robot_teams
        return config
