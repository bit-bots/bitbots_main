"""
Node that holds the global configuration (game settings, field dimensions, ...) shared by many nodes.

The parameters are available via the standard parameter services of the node and are additionally published as one
latched message on the global parameter blackboard topic, which contains the complete state of the blackboard.
The node also publishes the maps of the configured field, e.g. for the localization.
"""

import array
import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python import get_package_share_directory
from bitbots_utils.utils import (
    GLOBAL_PARAMETER_BLACKBOARD_TOPIC,
    LATCHED_QOS,
    get_parameters_from_ros_yaml,
)
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

from bitbots_msgs.msg import ParameterBlackboard as ParameterBlackboardMsg

NODE_NAME = "parameter_blackboard"

# Field maps that are published if they exist for the configured field, mapped to the name of their image file
FIELD_MAPS = {
    "lines": "lines.png",
    "goals": "goals.png",
}

# The field map images use one pixel per centimeter
FIELD_MAP_RESOLUTION = 0.01

# Parameters that the launch file sets on the node, as opposed to overrides of the configuration files
NODE_SETTINGS = {"field.name", "simulation_active", "use_sim_time"}

# Maximum intensity of a field map image, as the intensity is interpreted as an occupancy percentage
FIELD_MAP_MAX_INTENSITY = 100


def config_directory() -> Path:
    return Path(get_package_share_directory("bitbots_parameter_blackboard")) / "config"


def parameter_files(config_dir: Path, field_name: str, simulation_active: bool, domain_id: str | None) -> list[Path]:
    """
    Returns the parameter files that make up the global configuration.
    Later files take precedence over earlier ones.
    """
    files = [
        config_dir / "fields" / field_name / "config.yaml",
        config_dir / "global_parameters.yaml",
        config_dir / "game_settings.yaml",
    ]
    if simulation_active and domain_id is not None:
        # Each simulated robot runs in its own domain and gets its own game settings (e.g. its id)
        files.append(config_dir / f"sim_game_settings_{int(domain_id)}.yaml")
    return files


def load_parameter_files(files: list[Path]) -> dict[str, Parameter]:
    """
    Loads and merges the parameters for this node from the given ROS parameter files.

    :raises FileNotFoundError: If one of the files does not exist
    """
    parameters: dict[str, Parameter] = {}
    for file in files:
        if not file.is_file():
            raise FileNotFoundError(f"Global parameter file '{file}' does not exist")
        for parameter_msg in get_parameters_from_ros_yaml(NODE_NAME, str(file), use_wildcard=True):
            parameters[parameter_msg.name] = Parameter.from_parameter_msg(parameter_msg)
    return parameters


def check_overrides(overrides: dict[str, Parameter], file_parameters: dict[str, Parameter]) -> list[str]:
    """
    Returns why the given overrides are invalid.
    Overrides have to replace a parameter of the configuration files with a value of the same type,
    so that a typo can not silently add a new parameter or change the type of an existing one.
    """
    problems = []
    for name, override in sorted(overrides.items()):
        if name in NODE_SETTINGS:
            continue
        if name not in file_parameters:
            problems.append(f"'{name}' is not defined in any configuration file")
        elif override.type_ != file_parameters[name].type_:
            problems.append(
                f"'{name}' is a {file_parameters[name].type_.name} in the configuration files, "
                f"but the override is a {override.type_.name}"
            )
    return problems


def load_field_map(path: Path) -> np.ndarray:
    """
    Loads a field map image and converts it into occupancy values (row major, same layout as the image).

    :raises ValueError: If the image can not be read or exceeds the expected intensity range
    """
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"Field map '{path}' could not be read as a grayscale image")
    max_intensity = int(image.max())
    if max_intensity > FIELD_MAP_MAX_INTENSITY:
        raise ValueError(
            f"Field map '{path}' contains pixel intensities up to {max_intensity}, but intensities within "
            f"[0, {FIELD_MAP_MAX_INTENSITY}] are expected. If the map comes from the field map generator, "
            "convert it to the expected range."
        )
    return (FIELD_MAP_MAX_INTENSITY - image.astype(np.int16)).astype(np.int8)


def occupancy_grid_from_field_map(occupancy: np.ndarray, frame_id: str) -> OccupancyGrid:
    """
    Creates an occupancy grid message, which is centered on the field center, from occupancy values.
    """
    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    msg.info.resolution = FIELD_MAP_RESOLUTION
    msg.info.height, msg.info.width = occupancy.shape
    msg.info.origin.position.x = -msg.info.width / 2.0 * FIELD_MAP_RESOLUTION
    msg.info.origin.position.y = -msg.info.height / 2.0 * FIELD_MAP_RESOLUTION
    msg.info.origin.orientation.w = 1.0
    msg.data = array.array("b", np.ascontiguousarray(occupancy).tobytes())
    return msg


class ParameterBlackboard(Node):
    def __init__(self, **kwargs) -> None:
        # Undeclared parameters are allowed, so the blackboard can be extended at runtime like the standard one
        super().__init__(
            NODE_NAME, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True, **kwargs
        )

        if not self.has_parameter("field.name"):
            raise ValueError("The parameter 'field.name' is required to load the field configuration")
        self.field_name: str = self.get_parameter("field.name").get_parameter_value().string_value
        simulation_active = self.get_parameter_or(
            "simulation_active", Parameter("simulation_active", value=False)
        ).value

        # Load the configuration files, parameters given to the node directly (e.g. by the launch file) take precedence
        files = parameter_files(config_directory(), self.field_name, simulation_active, os.environ.get("ROS_DOMAIN_ID"))
        self.get_logger().info(f"Loading global parameters from {[str(file) for file in files]}")
        file_parameters = load_parameter_files(files)
        # The parameters passed to the node on startup (rclpy keeps them as overrides)
        overrides = {name: self.get_parameter(name) for name in self._parameter_overrides}
        problems = check_overrides(overrides, file_parameters)
        if problems:
            raise ValueError("Invalid overrides of global parameters: " + "; ".join(problems))
        for name in sorted(set(overrides) - NODE_SETTINGS):
            self.get_logger().info(f"Global parameter '{name}' is overridden with {overrides[name].value!r}")
        for parameter in file_parameters.values():
            if not self.has_parameter(parameter.name):
                self.declare_parameter(parameter.name, parameter.value)

        self.state_publisher = self.create_publisher(
            ParameterBlackboardMsg, GLOBAL_PARAMETER_BLACKBOARD_TOPIC, LATCHED_QOS
        )
        self.add_on_set_parameters_callback(self._validate_parameter_change)
        self.add_post_set_parameters_callback(lambda _: self._publish_state())
        self._publish_state()

        self.field_map_publishers = self._publish_field_maps()

    def _validate_parameter_change(self, parameters: list[Parameter]) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "field.name" and parameter.value != self.field_name:
                return SetParametersResult(
                    successful=False,
                    reason="The field can not be changed at runtime, restart the parameter blackboard instead",
                )
        return SetParametersResult(successful=True)

    def _publish_state(self) -> None:
        msg = ParameterBlackboardMsg()
        msg.stamp = self.get_clock().now().to_msg()
        msg.parameters = [
            parameter.to_parameter_msg()
            for parameter in sorted(self.get_parameters_by_prefix("").values(), key=lambda parameter: parameter.name)
            if parameter.type_ != Parameter.Type.NOT_SET
        ]
        self.state_publisher.publish(msg)

    def _publish_field_maps(self) -> list:
        map_frame: str = self.get_parameter("map_frame").get_parameter_value().string_value
        field_dir = config_directory() / "fields" / self.field_name
        publishers = []
        for map_name, file_name in FIELD_MAPS.items():
            path = field_dir / file_name
            topic = f"field/map/{map_name}"
            if not path.is_file():
                # Not every field provides every map, nodes that require a missing map report that they wait for it
                self.get_logger().info(
                    f"Field '{self.field_name}' has no {map_name} map at '{path}', not publishing '{topic}'"
                )
                continue
            msg = occupancy_grid_from_field_map(load_field_map(path), map_frame)
            msg.header.stamp = self.get_clock().now().to_msg()
            publisher = self.create_publisher(OccupancyGrid, topic, LATCHED_QOS)
            publisher.publish(msg)
            publishers.append(publisher)
            self.get_logger().info(f"Publishing the {map_name} map of field '{self.field_name}' on '{topic}'")
        return publishers


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParameterBlackboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
