"""Utilities for generating the MuJoCo world XML with a configurable robot setup.

The ``kid_field.xml`` template positions robots with a single ``<replicate>``
element that clones one robot with a uniform offset. That is not flexible enough
to place robots for a team versus team setup. Instead of encoding the placement
in the template, we parse the template, drop the ``<replicate>`` element and
insert one ``<frame>`` per robot so that every robot can be positioned
individually.
"""

import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

# Sideline offset from the field center along the y-axis.
SIDELINE_Y = 3.25
# Backward compatibility alias
TEAM_LINE_Y = SIDELINE_Y

# Half dimensions of the field (length 9m -> half length 4.5m).
HALF_FIELD_LENGTH = 4.5

# Clearance from the borders of the half (goal line at -4.5m/4.5m and midfield at 0.0m).
BORDER_CLEARANCE = 0.5

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

# RGB color palettes for teams and goalies.
DEFAULT_TEAM_COLORS: list[tuple[int, int, int]] = [
    (0, 70, 200),  # Team 0: Blue
    (200, 30, 30),  # Team 1: Red
]

DEFAULT_GOALIE_COLORS: list[tuple[int, int, int]] = [
    (230, 190, 20),  # Goalie 0: Yellow
    (30, 180, 60),  # Goalie 1: Green
]

# Directory where generated jersey textures are cached to avoid re-generating.
JERSEY_CACHE_DIR: Path = Path(tempfile.gettempdir()) / "bitbots_mujoco_sim_jerseys"


def get_robot_jersey_color(
    team_color: int,
    is_goalie: bool = False,
) -> tuple[int, int, int]:
    """Return the RGB jersey color for a robot based on its team and role."""
    if is_goalie:
        return DEFAULT_GOALIE_COLORS[team_color % len(DEFAULT_GOALIE_COLORS)]
    return DEFAULT_TEAM_COLORS[team_color % len(DEFAULT_TEAM_COLORS)]


def get_jersey_texture(
    color: tuple[int, int, int],
    number: int | str,
    cache_dir: Path = JERSEY_CACHE_DIR,
    size: int = 256,
) -> Path:
    """Generate and cache a jersey texture image with the specified color and number.

    The texture is cached in ``cache_dir`` (placed in a temporary directory without
    automatic deletion) and reused across simulation launches for performance.
    """
    hex_color = f"{color[0]:02x}{color[1]:02x}{color[2]:02x}"
    output_path = cache_dir / f"jersey_{hex_color}_{number}.png"

    if output_path.exists():
        return output_path

    output_path.parent.mkdir(parents=True, exist_ok=True)
    img = Image.new("RGB", (size, size), color=color)
    draw = ImageDraw.Draw(img)

    # Choose contrasting text color based on background luminance
    r, g, b = color
    luminance = 0.299 * r + 0.587 * g + 0.114 * b
    text_color = (0, 0, 0) if luminance > 140 else (255, 255, 255)

    font_size = int(size * 0.6)
    try:
        font = ImageFont.load_default(size=font_size)
    except TypeError:
        font = ImageFont.load_default()

    draw.text((size // 2, size // 2), str(number), fill=text_color, font=font, anchor="mm")
    img.save(output_path)
    return output_path


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

    Robots are dynamically distributed evenly along the available sideline space
    of their own half, keeping at least 0.5m clearance from borders.
    - Position 1 is placed on the left sideline, position 2 on the right sideline.
    - Unassigned / center positions are balanced across both sides so player
      counts per sideline are as even as possible.
    - On each sideline, robots are sorted from back to front according to role:
      goalie at the back, offense at the front, with center defense/offense
      positioned in front of side defense/offense.

    For team 0 (defending negative x half, facing +x), left is +y and right is -y.
    For team 1 (defending positive x half, facing -x), poses are mirrored across
    the field origin (left is -y and right is +y).
    """
    poses: list[tuple[float, float, float]] = []
    game_settings = compute_game_settings(teams)

    x_min = -(HALF_FIELD_LENGTH - BORDER_CLEARANCE)
    x_max = -BORDER_CLEARANCE

    def compute_x_positions(k: int) -> list[float]:
        if k <= 0:
            return []
        if k == 1:
            return [(x_min + x_max) / 2.0]
        return [x_min + i * (x_max - x_min) / (k - 1) for i in range(k)]

    def role_sort_key(item: tuple[int, dict]) -> int:
        _idx, setting = item
        role = setting["role"]
        pos = setting["position_number"]
        if role == "goalie":
            return 0
        elif role == "defense":
            return 2 if pos == 0 else 1
        elif role == "offense":
            return 4 if pos == 0 else 3
        return 0

    for team_index, count in enumerate(teams):
        team_start = sum(teams[:team_index])
        team_settings = game_settings[team_start : team_start + count]
        team_robots = list(enumerate(team_settings))

        left_robots: list[tuple[int, dict]] = []
        right_robots: list[tuple[int, dict]] = []
        unassigned_robots: list[tuple[int, dict]] = []

        for idx, setting in team_robots:
            pos_num = setting["position_number"]
            if pos_num == 1:
                left_robots.append((idx, setting))
            elif pos_num == 2:
                right_robots.append((idx, setting))
            else:
                unassigned_robots.append((idx, setting))

        # Fill up unassigned so that left and right are about even
        for idx, setting in unassigned_robots:
            if len(left_robots) < len(right_robots):
                left_robots.append((idx, setting))
            elif len(right_robots) < len(left_robots):
                right_robots.append((idx, setting))
            else:
                right_robots.append((idx, setting))

        # Sort according to role from back to front
        left_robots.sort(key=role_sort_key)
        right_robots.sort(key=role_sort_key)

        left_x = compute_x_positions(len(left_robots))
        right_x = compute_x_positions(len(right_robots))

        team_poses: list[tuple[float, float, float]] = [None] * count  # type: ignore[list-item]

        for (idx, _setting), x_base in zip(left_robots, left_x, strict=True):
            if team_index % 2 == 0:
                x = x_base
                y = SIDELINE_Y
            else:
                x = -x_base
                y = -SIDELINE_Y
            yaw = FACING_YAW if y < 0 else -FACING_YAW
            team_poses[idx] = (x, y, yaw)

        for (idx, _setting), x_base in zip(right_robots, right_x, strict=True):
            if team_index % 2 == 0:
                x = x_base
                y = -SIDELINE_Y
            else:
                x = -x_base
                y = SIDELINE_Y
            yaw = FACING_YAW if y < 0 else -FACING_YAW
            team_poses[idx] = (x, y, yaw)

        poses.extend(team_poses)

    return poses


def _find_replicate(root: ET.Element) -> tuple[ET.Element, ET.Element]:
    """Return the ``<replicate>`` element and its parent from the world tree."""
    for parent in root.iter():
        for child in parent:
            if child.tag == "replicate":
                return parent, child
    raise ValueError("No <replicate> element found in world template")


def generate_world_xml(
    num_robots: str | int,
    package_share: str,
    robot_type: str,
) -> Path:
    """Generate the MuJoCo world XML positioning every robot individually.

    The ``<replicate>`` element of the template is replaced by one ``<frame>`` per
    robot, each attaching the robot model at its computed pose. Every robot gets a
    unique attach prefix (``robot_<index>_``) so that the resulting body names stay
    unique and can be discovered by the simulation.

    Additionally, per-robot jersey textures and materials are dynamically generated,
    cached in a temporary directory, and registered in the model assets so that each
    robot displays its team/goalie color and player number in both the visualizer and
    camera images.

    Returns the path of the generated world file.
    """
    template_path = Path(package_share) / "xml" / "kid_field.xml"
    output_path = Path(package_share) / "xml" / "generated_world.xml"

    teams = parse_num_robots(num_robots)
    poses = compute_robot_poses(teams)
    game_settings = compute_game_settings(teams)

    tree = ET.parse(template_path)
    root = tree.getroot()

    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")

    replicate_parent, replicate = _find_replicate(root)
    insert_index = list(replicate_parent).index(replicate)
    replicate_parent.remove(replicate)

    for robot_index, ((x, y, yaw), setting) in enumerate(zip(poses, game_settings, strict=False)):
        is_goalie = setting["role"] == "goalie"
        color = get_robot_jersey_color(
            setting["team_color"],
            is_goalie=is_goalie,
        )
        texture_path = get_jersey_texture(color, setting["bot_id"])

        tex_name = f"robot_{robot_index}_jersey_override"
        mat_name = f"robot_{robot_index}_jersey_override"
        ET.SubElement(asset, "texture", {"name": tex_name, "type": "2d", "file": str(texture_path)})
        ET.SubElement(asset, "material", {"name": mat_name, "texture": tex_name})

        frame = ET.Element("frame", {"pos": f"{x} {y} 0.0", "euler": f"0 0 {yaw}"})
        ET.SubElement(frame, "attach", {"model": robot_type, "prefix": f"robot_{robot_index}_"})
        replicate_parent.insert(insert_index, frame)
        insert_index += 1

    tree.write(output_path, encoding="unicode", xml_declaration=False)
    return output_path
