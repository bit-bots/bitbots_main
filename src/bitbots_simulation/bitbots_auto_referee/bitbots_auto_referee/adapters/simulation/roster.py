"""Validate the running world's roster against the referee configuration."""

from typing import TYPE_CHECKING

from bitbots_auto_referee.adapters.simulation.lineup import jersey_color

if TYPE_CHECKING:
    from bitbots_auto_referee.config import RefereeConfig


def validate_roster(config: "RefereeConfig", roster: list[dict]) -> dict[int, dict]:
    if not isinstance(roster, list) or not roster:
        raise ValueError("Simulation roster must be a nonempty list")
    robots = {}
    domains = set()
    for robot in roster:
        if not isinstance(robot, dict):
            raise ValueError("Invalid simulation roster entry")
        for field in ("index", "team_id", "bot_id", "team_color", "domain"):
            if type(robot.get(field)) is not int or robot[field] < 0:
                raise ValueError(f"Invalid roster field {field}")
        index = robot["index"]
        if index in robots or robot["domain"] in domains:
            raise ValueError("Duplicate robot index or domain in simulation roster")
        robots[index] = robot
        domains.add(robot["domain"])
    actual = {index: (robot["team_id"], robot["bot_id"]) for index, robot in robots.items()}
    expected = {index: (team, config.robot_players[index]) for index, team in config.robot_teams.items()}
    if actual != expected:
        raise ValueError(f"Robot/team assignment mismatch: simulation={actual}, AutoRef={expected}; check num_robots")

    from bitbots_mujoco_sim.world import get_robot_jersey_color

    for index, robot in robots.items():
        if robot.get("role") not in ("offense", "defense", "goalie"):
            raise ValueError(f"Invalid role for robot {index}")
        if index in config.robot_roles and config.robot_roles[index] != robot["role"]:
            raise ValueError(f"Role mismatch for robot {index}")
        side = "home" if robot["team_id"] == config.home_team_id else "away"
        goalkeeper = getattr(config, f"{side}_goalkeeper")
        if (robot["role"] == "goalie") != (robot["bot_id"] == goalkeeper):
            raise ValueError(f"Goalkeeper mismatch for robot {index}")
        for suffix, is_goalie in (("color", False), ("goalkeeper_color", True)):
            color = jersey_color(get_robot_jersey_color(robot["team_color"], is_goalie=is_goalie))
            if getattr(config, f"{side}_{suffix}") != color:
                raise ValueError(f"Team color mismatch for robot {index}")
    return robots
