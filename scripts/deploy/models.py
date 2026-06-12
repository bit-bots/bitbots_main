from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

TEAMPLAYER_COMPONENTS = (
    "lowlevel",
    "motion",
    "game_controller",
    "vision",
    "ipm",
    "localization",
    "path_planning",
    "behavior",
    "teamcom",
    "world_model",
    "whistle_detector",
    "audio",
    "tts",
    "monitoring",
    "record",
)

DEFAULT_COMPONENTS = frozenset(
    {
        "lowlevel",
        "motion",
        "game_controller",
        "vision",
        "ipm",
        "localization",
        "path_planning",
        "behavior",
        "teamcom",
        "world_model",
        "whistle_detector",
        "audio",
    }
)


class Step(str, Enum):
    CONNECT = "connect"
    ENVIRONMENT = "environment"
    SOURCE = "source"
    CONFIGURATION = "configuration"
    BUILD = "build"
    WIFI = "wifi"
    TEAMPLAYER = "teamplayer"


class StepState(str, Enum):
    PENDING = "pending"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    WAITING = "waiting"
    SKIPPED = "skipped"


@dataclass(frozen=True)
class RobotTarget:
    name: str
    host: str
    robot_name: str = ""
    user: str = "nvidia"
    workspace: str = "~/bitbots/bitbots_main"

    @property
    def destination(self) -> str:
        return f"{self.user}@{self.host}"


@dataclass(frozen=True)
class MatchProfile:
    name: str
    field: str
    ball: str
    wifi_profile: str
    game_settings: dict[str, Any]


@dataclass(frozen=True)
class BallProfile:
    name: str
    diameter: float


@dataclass(frozen=True)
class DesiredState:
    generation: int
    match: MatchProfile
    ball: BallProfile
    components: frozenset[str] = frozenset()
    sync_source: bool = True
    build: bool = True
    build_packages: tuple[str, ...] = ()
    clean_build: bool = False
    launch: bool = True


@dataclass
class CommandResult:
    command: tuple[str, ...]
    returncode: int
    stdout: str = ""
    stderr: str = ""

    @property
    def ok(self) -> bool:
        return self.returncode == 0

    @property
    def final_error(self) -> str:
        lines = [line.strip() for line in self.stderr.splitlines() if line.strip()]
        if not lines:
            lines = [line.strip() for line in self.stdout.splitlines() if line.strip()]
        return lines[-1] if lines else f"command exited with status {self.returncode}"


class CommandFailedError(RuntimeError):
    def __init__(self, result: CommandResult, transient: bool = False) -> None:
        self.result = result
        self.transient = transient
        super().__init__(f"{' '.join(result.command)} exited {result.returncode}: {result.final_error}")


@dataclass
class StepReport:
    state: StepState = StepState.PENDING
    summary: str = ""
    details: str = ""


@dataclass
class RobotStatus:
    target: RobotTarget
    connected: bool = False
    generation: int = 0
    applied_generation: int = 0
    source_fingerprint: str = ""
    applied_source_fingerprint: str = ""
    teamplayer: dict[str, Any] = field(default_factory=dict)
    steps: dict[Step, StepReport] = field(default_factory=lambda: {step: StepReport() for step in Step})
    logs: list[str] = field(default_factory=list)

    @property
    def stale(self) -> bool:
        return bool(self.source_fingerprint) and self.source_fingerprint != self.applied_source_fingerprint

    def append_log(self, line: str, limit: int = 4000) -> None:
        self.logs.append(line.rstrip())
        if len(self.logs) > limit:
            del self.logs[: len(self.logs) - limit]
