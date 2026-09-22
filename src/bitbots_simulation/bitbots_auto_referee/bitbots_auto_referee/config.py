"""Shared launch defaults and validated startup configuration."""

import ipaddress
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
    "use_sim_time": ParameterSpec(True, "Use the simulator's clock for the opening sequence and referee decisions."),
}


@dataclass(frozen=True)
class RefereeConfig:
    league_size: str
    lineup_mode: str
    home_team_id: int
    away_team_id: int
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
                raise ValueError(f"{name} must be a valid UDP port")
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

        fields = {name: value for name, value in values.items() if name not in ("leagueSize", "use_sim_time")}
        return cls(league_size=values["leagueSize"], **fields)
