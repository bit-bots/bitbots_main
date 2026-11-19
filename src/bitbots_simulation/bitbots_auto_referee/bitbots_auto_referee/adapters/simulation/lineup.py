"""Resolve referee identities from the simulator's world configuration."""

import colorsys
import json


def jersey_color(rgb: tuple[int, int, int]) -> str:
    """Translate the simulator palette into the GameController color vocabulary."""
    hue, saturation, value = colorsys.rgb_to_hsv(*(component / 255 for component in rgb))
    if value < 0.2:
        return "black"
    if saturation < 0.2:
        return "white" if value > 0.8 else "gray"
    degrees = hue * 360
    if degrees < 15 or degrees >= 345:
        return "red"
    if degrees < 40:
        return "brown" if value < 0.65 else "orange"
    if degrees < 75:
        return "yellow"
    if degrees < 170:
        return "green"
    if degrees < 260:
        return "blue"
    return "purple"


def apply_simulation_lineup(values: dict) -> dict[int, str]:
    """Use the same robot order, identities, roles and palette as world generation.

    An empty setup retains explicit referee parameters for standalone operation.
    The world helper becomes available when the multi-team branch is merged.
    """
    if not values["num_robots"]:
        return {}
    try:
        from bitbots_mujoco_sim import world
    except ImportError as error:
        raise ValueError("num_robots requires the multi-team simulator's world configuration") from error

    teams = world.parse_num_robots(values["num_robots"])
    if len(teams) > 2:
        raise ValueError("The GameController protocol supports only home and away teams")
    settings = world.compute_game_settings(teams)
    values["home_team_id"] = world.BASE_TEAM_ID
    values["away_team_id"] = world.BASE_TEAM_ID + 1
    values["home_defends_negative_x"] = True
    robot_teams, robot_players, roles = {}, {}, {}
    for side, count in zip(("home", "away"), teams + [0] * (2 - len(teams)), strict=True):
        team_index = 0 if side == "home" else 1
        team_settings = settings[sum(teams[:team_index]) : sum(teams[:team_index]) + count]
        values[f"{side}_goalkeeper"] = next(
            (robot["bot_id"] for robot in team_settings if robot["role"] == "goalie"), 0
        )
        color = team_settings[0]["team_color"] if team_settings else team_index % 2
        values[f"{side}_color"] = jersey_color(world.get_robot_jersey_color(color))
        values[f"{side}_goalkeeper_color"] = jersey_color(world.get_robot_jersey_color(color, is_goalie=True))
        for index, robot in enumerate(team_settings, sum(teams[:team_index])):
            robot_teams[str(index)] = side
            robot_players[str(index)] = robot["bot_id"]
            roles[index] = robot["role"]
    values["robot_team_mapping"] = json.dumps(robot_teams)
    values["robot_player_mapping"] = json.dumps(robot_players)
    return roles
