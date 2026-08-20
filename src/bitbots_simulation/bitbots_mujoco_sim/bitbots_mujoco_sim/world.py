"""Utilities for generating the MuJoCo world XML with a configurable robot setup.

The ``kid_field.xml`` template positions robots with a single ``<replicate>``
element that clones one robot with a uniform offset. That is not flexible enough
to place robots for a team versus team setup. Instead of encoding the placement
in the template, we parse the template, drop the ``<replicate>`` element and
insert one ``<frame>`` per robot so that every robot can be positioned
individually.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

# Sideline offset from the field center along the y-axis.
SIDELINE_Y = 3.25
# Backward compatibility alias
TEAM_LINE_Y = SIDELINE_Y

# Half dimensions of the field (length 9m -> half length 4.5m).
HALF_FIELD_LENGTH = 4.5

# Yaw (rotation about z) to make a robot face inward towards the field center.
# A robot on the negative-y sideline faces +y; on the positive-y sideline it faces -y.
FACING_YAW = 1.57

# Role assigned to a robot based on its position within a team (0-based). A team
# has at most 3 offense, 3 defense and 1 goalie players (7 players total). Roles
# are filled in this fixed order, and the position_number within each of the
# offense/defense groups is assigned in the order the robots appear (0, 1, 2).
TEAM_ROLE_ORDER = ["offense", "goalie", "defense", "defense", "defense", "offense", "offense"]

# Default team ID for the first team. Additional teams increment from here.
BASE_TEAM_ID = 6

# Fixed placement along the sideline for team 0 (defending x in [-4.5, 0], facing +x).
# Offense is placed at the front (near x=0), goalie at the back (near x=-4.5), and
# defenders in the middle. Position 1 (left) goes to the left sideline (+y),
# position 2 (right) goes to the right sideline (-y), and center (position 0)
# positions are distributed across sides (offense/goalie right, defense left).
# All positions leave >= 0.5m space from the borders of the half (-4.5 and 0.0).
# Formatted as: (role, position_number) -> (x_pos, side) where side is "left" or "right".
_ROLE_PLACEMENTS_TEAM_0: dict[tuple[str, int], tuple[float, str]] = {
    # Offense (front): position 0 = center (right side), 1 = left, 2 = right
    ("offense", 0): (-0.6, "right"),
    ("offense", 1): (-0.6, "left"),
    ("offense", 2): (-1.2, "right"),
    # Goalie (back): position 0 = right side
    ("goalie", 0): (-3.9, "right"),
    # Defense (middle): position 0 = center (left side), 1 = left, 2 = right
    ("defense", 0): (-2.6, "left"),
    ("defense", 1): (-2.3, "left"),
    ("defense", 2): (-2.0, "right"),
}


def parse_num_robots(num_robots: str | int) -> list[int]:
    """Parse a ``num_robots`` specification into per-team robot counts.

    Accepts either a single number (e.g. ``"3"`` -> a single team of three
    robots) or a colon separated team setup (e.g. ``"2:2"`` -> two teams of two
    robots each). Returns the robot count of every team in order.
    """
    teams = [int(part) for part in str(num_robots).split(":")]
    if any(count < 0 for count in teams) or sum(teams) < 1:
        raise ValueError(f"Invalid num_robots specification: {num_robots!r}")
    return teams


def compute_game_settings(teams: list[int]) -> list[dict]:
    """Compute the per-robot game settings for a team setup.

    Given the per-team robot counts (e.g. ``[2, 2]`` for a 2 vs 2), this returns
    one dict of game settings per robot, ordered team by team. ``bot_id`` is unique
    within a team (1-based) while ``team_id``/``team_color`` identify the team, so
    any team layout works without hand-maintained per-robot config files.

    Roles are assigned per team following ``TEAM_ROLE_ORDER`` and the
    ``position_number`` is counted separately per role group so that it is never
    duplicated within a team (e.g. the three offense players get positions 0, 1, 2).
    """
    settings: list[dict] = []
    for team_index, count in enumerate(teams):
        if count > len(TEAM_ROLE_ORDER):
            raise ValueError(
                f"Team {team_index} has {count} robots, but at most {len(TEAM_ROLE_ORDER)} players per team are supported"
            )
        role_position_counters: dict[str, int] = {}
        for robot_in_team in range(count):
            role = TEAM_ROLE_ORDER[robot_in_team]
            position_number = role_position_counters.get(role, 0)
            role_position_counters[role] = position_number + 1
            settings.append(
                {
                    "bot_id": robot_in_team + 1,
                    "team_id": BASE_TEAM_ID + team_index,
                    "team_color": team_index % 2,
                    "role": role,
                    "position_number": position_number,
                }
            )
    return settings


def compute_robot_poses(teams: list[int]) -> list[tuple[float, float, float]]:
    """Compute the ``(x, y, yaw)`` placement for every robot.

    Robots are positioned on the sidelines of their own half. Offense players are
    placed to the front (near midfield), the goalie to the back (near goal line),
    and defenders in the middle. Role position 1 is left and position 2 is right
    (relative to local team orientation), while position 0 is placed on the side.
    All positions keep at least 0.5m clearance from the borders of the half.

    For team 0 (defending negative x half, facing +x), left is +y and right is -y.
    For team 1 (defending positive x half, facing -x), poses are mirrored across
    the field origin (left is -y and right is +y).
    """
    poses: list[tuple[float, float, float]] = []
    game_settings = compute_game_settings(teams)

    for team_index, count in enumerate(teams):
        # Determine team offset in the flat game_settings list
        team_start = sum(teams[:team_index])
        for robot_in_team in range(count):
            setting = game_settings[team_start + robot_in_team]
            role = setting["role"]
            pos_num = setting["position_number"]

            x_base, side = _ROLE_PLACEMENTS_TEAM_0[(role, pos_num)]

            if team_index % 2 == 0:
                # Team 0: defending -x half, facing +x
                x = x_base
                y = SIDELINE_Y if side == "left" else -SIDELINE_Y
            else:
                # Team 1: defending +x half, facing -x (mirrored across origin)
                x = -x_base
                y = -SIDELINE_Y if side == "left" else SIDELINE_Y

            # Facing angle: turn towards field center (y = 0)
            yaw = FACING_YAW if y < 0 else -FACING_YAW
            poses.append((x, y, yaw))

    return poses


def _find_replicate(root: ET.Element) -> tuple[ET.Element, ET.Element]:
    """Return the ``<replicate>`` element and its parent from the world tree."""
    for parent in root.iter():
        for child in parent:
            if child.tag == "replicate":
                return parent, child
    raise ValueError("No <replicate> element found in world template")


def generate_world_xml(num_robots: str | int, package_share: str, robot_type: str) -> Path:
    """Generate the MuJoCo world XML positioning every robot individually.

    The ``<replicate>`` element of the template is replaced by one ``<frame>`` per
    robot, each attaching the robot model at its computed pose. Every robot gets a
    unique attach prefix (``robot_<index>_``) so that the resulting body names stay
    unique and can be discovered by the simulation.

    Returns the path of the generated world file.
    """
    template_path = Path(package_share) / "xml" / "kid_field.xml"
    output_path = Path(package_share) / "xml" / "generated_world.xml"

    poses = compute_robot_poses(parse_num_robots(num_robots))

    tree = ET.parse(template_path)
    replicate_parent, replicate = _find_replicate(tree.getroot())
    insert_index = list(replicate_parent).index(replicate)
    replicate_parent.remove(replicate)

    for robot_index, (x, y, yaw) in enumerate(poses):
        frame = ET.Element("frame", {"pos": f"{x} {y} 0.0", "euler": f"0 0 {yaw}"})
        ET.SubElement(frame, "attach", {"model": robot_type, "prefix": f"robot_{robot_index}_"})
        replicate_parent.insert(insert_index, frame)
        insert_index += 1

    tree.write(output_path, encoding="unicode", xml_declaration=False)
    return output_path
