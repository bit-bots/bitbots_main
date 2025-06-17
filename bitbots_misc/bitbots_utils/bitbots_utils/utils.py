import os
from threading import Thread
from typing import Any

import rclpy
import yaml
from ament_index_python import get_package_share_directory
from beartype import BeartypeConf, BeartypeStrategy, beartype
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType as ParameterTypeMsg
from rcl_interfaces.msg import ParameterValue as ParameterValueMsg
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import PARAMETER_SEPARATOR_STRING, Parameter, parameter_value_to_python
from rclpy.task import Future

# Create a new @nobeartype decorator disabling type-checking.
nobeartype = beartype(conf=BeartypeConf(strategy=BeartypeStrategy.O0))


def read_urdf(robot_name: str) -> str:
    urdf = os.popen(
        f"xacro {get_package_share_directory(f'{robot_name}_description')}"
        f"/urdf/robot.urdf use_fake_walk:=false sim_ns:=false"
    ).read()
    return urdf


def load_moveit_parameter(robot_name: str) -> list[ParameterMsg]:
    moveit_parameters = get_parameters_from_plain_yaml(
        f"{get_package_share_directory(f'{robot_name}_moveit_config')}/config/kinematics.yaml",
        "robot_description_kinematics.",
    )
    robot_description = ParameterMsg()
    robot_description.name = "robot_description"
    robot_description.value = ParameterValueMsg(
        string_value=read_urdf(robot_name), type=ParameterTypeMsg.PARAMETER_STRING
    )
    moveit_parameters.append(robot_description)
    robot_description_semantic = ParameterMsg()
    robot_description_semantic.name = "robot_description_semantic"
    with open(f"{get_package_share_directory(f'{robot_name}_moveit_config')}/config/{robot_name}.srdf") as file:
        value = file.read()
        robot_description_semantic.value = ParameterValueMsg(string_value=value, type=ParameterTypeMsg.PARAMETER_STRING)
    moveit_parameters.append(robot_description_semantic)
    return moveit_parameters


def get_parameters_from_ros_yaml(node_name: str, parameter_file: str, use_wildcard: bool) -> list[ParameterMsg]:
    # Remove leading slash and namespaces
    with open(parameter_file) as f:
        param_file = yaml.safe_load(f)
        param_keys = []
        if use_wildcard and "/**" in param_file:
            param_keys.append("/**")
        if node_name in param_file:
            param_keys.append(node_name)

        if param_keys == []:
            raise RuntimeError(
                f"Param file does not contain parameters for {node_name},  only for nodes: {param_file.keys()}"
            )
        param_dict = {}
        for k in param_keys:
            value = param_file[k]
            if not isinstance(value, dict) or "ros__parameters" not in value:
                raise RuntimeError(
                    f"Invalid structure of parameter file for node {k}"
                    "expected same format as provided by ros2 param dump"
                )
            param_dict.update(value["ros__parameters"])
        return parse_parameter_dict(namespace="", parameter_dict=param_dict)


def get_parameters_from_plain_yaml(parameter_file, namespace="") -> list[ParameterMsg]:
    with open(parameter_file) as f:
        param_dict = yaml.safe_load(f)
        return parse_parameter_dict(namespace=namespace, parameter_dict=param_dict)


def get_parameter_dict(node: Node, prefix: str) -> dict:
    """
    Get a dictionary of parameters from a node.

    :param node: Node to get parameters from
    :param prefix: Prefix for the nesting of the parameter query (e.g. 'body.nice_param.index' could have the prefix 'body')
    :return: Dictionary of parameters without prefix nesting
    """
    parameter_config: dict[str, Parameter] = node.get_parameters_by_prefix(prefix)
    config = dict()
    for param in parameter_config.values():
        # Split separated keys into nested dicts
        param_nests = param.name[len(prefix) :].split(PARAMETER_SEPARATOR_STRING)
        # Remove empty first element from split (because of possible leading separator)
        if param_nests[0] == "":
            param_nests.pop(0)
        # Traverse nested dicts and create them if they don't exist
        param_dict = config
        # Iterate all nests
        for i, nest in enumerate(param_nests):
            # If last nest, set value
            if i == len(param_nests) - 1:
                param_dict[nest] = param.value
            else:
                # If nest doesn't exist, create it
                if nest not in param_dict:
                    # Create nested dict if nest is not last
                    param_dict[nest] = dict()
                param_dict = param_dict[nest]
    return config


def get_parameters_from_other_node(
    own_node: Node, other_node_name: str, parameter_names: list[str], service_timeout_sec: float = 20.0
) -> dict:
    """
    Used to receive parameters from other running nodes.
    Returns a dict with requested parameter name as dict key and parameter value as dict value.
    """
    client = own_node.create_client(GetParameters, f"{other_node_name}/get_parameters")
    ready = client.wait_for_service(timeout_sec=service_timeout_sec)
    if not ready:
        raise RuntimeError(f"Wait for {other_node_name} parameter service timed out")
    request = GetParameters.Request()
    request.names = parameter_names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(own_node, future)
    response = future.result()

    results = {}  # Received parameter
    for i, param in enumerate(parameter_names):
        results[param] = parameter_value_to_python(response.values[i])
    return results


def get_parameters_from_other_node_sync(
    own_node: Node, other_node_name: str, parameter_names: list[str], service_timeout_sec: float = 20.0
) -> dict:
    """
    Used to receive parameters from other running nodes. It does not use async internally.
    It should not be used in callback functions, but it is a bit more reliable than the async version.
    Returns a dict with requested parameter name as dict key and parameter value as dict value.
    """
    client = own_node.create_client(GetParameters, f"{other_node_name}/get_parameters")
    ready = client.wait_for_service(timeout_sec=service_timeout_sec)
    if not ready:
        raise RuntimeError(f"Wait for {other_node_name} parameter service timed out")
    request = GetParameters.Request()
    request.names = parameter_names
    response = client.call(request)

    results = {}  # Received parameter
    for i, param in enumerate(parameter_names):
        results[param] = parameter_value_to_python(response.values[i])
    return results


def set_parameters_of_other_node(
    own_node: Node,
    other_node_name: str,
    parameter_names: list[str],
    parameter_values: list[Any],
    service_timeout_sec: float = 20.0,
) -> list[bool]:
    """
    Used to set parameters of another running node.
    Returns a list of booleans indicating success or failure.
    """
    client = own_node.create_client(SetParameters, f"{other_node_name}/set_parameters")
    ready = client.wait_for_service(timeout_sec=service_timeout_sec)
    if not ready:
        raise RuntimeError(f"Wait for {other_node_name} parameter service timed out")
    request = SetParameters.Request()

    for name, value in zip(parameter_names, parameter_values):
        param = Parameter(name=name, value=value)
        parameter_value_msg = param.to_parameter_msg()
        request.parameters.append(parameter_value_msg)

    future = client.call_async(request)
    rclpy.spin_until_future_complete(own_node, future)
    response = future.result()
    return [res.success for res in response.results]


def parse_parameter_dict(*, namespace: str, parameter_dict: dict) -> list[ParameterMsg]:
    parameters = []
    for param_name, param_value in parameter_dict.items():
        full_param_name = namespace + param_name
        # Unroll nested parameters
        if isinstance(param_value, dict):
            parameters += parse_parameter_dict(
                namespace=full_param_name + PARAMETER_SEPARATOR_STRING, parameter_dict=param_value
            )
        else:
            parameter = Parameter(name=full_param_name, value=param_value)
            parameters.append(parameter.to_parameter_msg())
    return parameters


async def async_wait_for(node: Node, rel_time: float):
    """
    ROS2 does not provide an async sleep function, so we implement our own using a timer.
    This function will wait for the specified relative time in seconds.

    :param node: The ROS2 node to create the timer on.
    :param rel_time: The relative time in seconds to wait.
    :return: None
    """
    future = Future()
    rel_time = max(rel_time, 0.0)

    def done_waiting():
        future.set_result(None)

    timer = node.create_timer(rel_time, done_waiting, clock=node.get_clock())
    await future
    timer.cancel()
    node.destroy_timer(timer)


async def async_run_thread(func: callable) -> None:
    """
    Allows the usage of blocking functions in an async context.

    Spawns a thread to run the function and returns a Future that will be set when the function is done.
    """
    future = Future()
    thread = Thread(target=lambda: future.set_result(func()))
    thread.start()
    await future
