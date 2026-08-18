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

# Robots of a team are lined up parallel to the goal line, offset from the field
# center along the y-axis. The value mirrors the sideline offset that was
# previously baked into the single-team template.
TEAM_LINE_Y = 3.25
# Distance between adjacent robots within a single team row, along the x-axis.
ROBOT_ROW_SPACING = 1.0
# Yaw (rotation about z) that makes a robot on the negative-y line face the field
# center. Robots on the positive-y line face the opposite direction.
FACING_YAW = 1.57


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


def compute_robot_poses(teams: list[int]) -> list[tuple[float, float, float]]:
    """Compute the ``(x, y, yaw)`` placement for every robot.

    Robots of a team are spread along the x-axis and centered on x=0. With more
    than one team the teams are distributed across the width of the field along
    the y-axis and turned to face each other. The returned poses are ordered team
    by team, matching the order of ``teams``.
    """
    poses: list[tuple[float, float, float]] = []
    num_teams = len(teams)
    for team_index, count in enumerate(teams):
        if num_teams == 1:
            y = -TEAM_LINE_Y
        else:
            # Distribute the teams evenly between the two team lines.
            fraction = team_index / (num_teams - 1)
            y = -TEAM_LINE_Y + fraction * (2 * TEAM_LINE_Y)
        # Robots on (or below) the center line face +y, the others face -y.
        yaw = FACING_YAW if y <= 0 else -FACING_YAW
        for robot_in_team in range(count):
            x = (robot_in_team - (count - 1) / 2.0) * ROBOT_ROW_SPACING
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
