from __future__ import annotations

import ipaddress
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from deploy.models import BallProfile, MatchProfile, RobotTarget


class ProfileError(ValueError):
    pass


@dataclass(frozen=True)
class ProfileStore:
    targets: dict[str, RobotTarget]
    matches: dict[str, MatchProfile]
    balls: dict[str, BallProfile]

    @classmethod
    def load(cls, deploy_dir: Path) -> ProfileStore:
        targets = _load_yaml(deploy_dir / "known_targets.yaml")
        profiles = _load_yaml(deploy_dir / "profiles.yaml")
        balls = _parse_balls(profiles.get("balls", {}))
        matches = _parse_matches(profiles.get("matches", {}))
        for match in matches.values():
            if match.ball not in balls:
                raise ProfileError(f"Match profile {match.name} references unknown ball profile {match.ball}")
        return cls(
            targets=_parse_targets(targets),
            matches=matches,
            balls=balls,
        )

    def resolve_targets(self, names: list[str], user: str, workspace: str) -> list[RobotTarget]:
        if not names:
            return [RobotTarget(t.name, t.host, t.robot_name, user, workspace) for t in self.targets.values()]

        aliases: dict[str, RobotTarget] = {}
        for target in self.targets.values():
            aliases[target.name] = target
            aliases[target.host] = target
            if target.robot_name:
                aliases[target.robot_name] = target

        resolved: list[RobotTarget] = []
        requested = list(self.targets) if any(name.upper() == "ALL" for name in names) else names
        for name in requested:
            target = aliases.get(name)
            if target is None:
                target = RobotTarget(name=name, host=name)
            resolved.append(RobotTarget(target.name, target.host, target.robot_name, user, workspace))
        return list(dict.fromkeys(resolved))


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as exc:
        raise ProfileError(f"Could not load {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ProfileError(f"{path} must contain a YAML mapping")
    return data


def _parse_targets(data: dict[str, Any]) -> dict[str, RobotTarget]:
    targets: dict[str, RobotTarget] = {}
    for host, raw in data.items():
        if not isinstance(raw, dict):
            raise ProfileError(f"Target {host} must be a mapping")
        ipaddress.ip_address(host)
        name = str(raw.get("hostname", host))
        targets[name] = RobotTarget(
            name=name,
            host=host,
            robot_name=str(raw.get("robot_name", "")),
        )
    return targets


def _parse_balls(data: dict[str, Any]) -> dict[str, BallProfile]:
    balls: dict[str, BallProfile] = {}
    for name, raw in data.items():
        if not isinstance(raw, dict) or not isinstance(raw.get("diameter"), (int, float)):
            raise ProfileError(f"Ball profile {name} requires a numeric diameter")
        diameter = float(raw["diameter"])
        if diameter <= 0:
            raise ProfileError(f"Ball profile {name} diameter must be positive")
        balls[name] = BallProfile(name=name, diameter=diameter)
    if not balls:
        raise ProfileError("At least one ball profile is required")
    return balls


def _parse_matches(data: dict[str, Any]) -> dict[str, MatchProfile]:
    matches: dict[str, MatchProfile] = {}
    required_settings = {
        "team_id": int,
        "team_color": int,
        "role": str,
        "position_number": int,
        "monitoring_host_ip": str,
    }
    for name, raw in data.items():
        if not isinstance(raw, dict):
            raise ProfileError(f"Match profile {name} must be a mapping")
        settings = raw.get("game_settings", {})
        if not isinstance(settings, dict):
            raise ProfileError(f"Match profile {name} game_settings must be a mapping")
        for key, value_type in required_settings.items():
            if key not in settings or not isinstance(settings[key], value_type):
                raise ProfileError(f"Match profile {name} game_settings.{key} must be {value_type.__name__}")
        ipaddress.ip_address(settings["monitoring_host_ip"])
        matches[name] = MatchProfile(
            name=name,
            field=str(raw["field"]),
            ball=str(raw["ball"]),
            wifi_profile=str(raw["wifi_profile"]),
            game_settings=settings,
        )
    if not matches:
        raise ProfileError("At least one match profile is required")
    return matches
