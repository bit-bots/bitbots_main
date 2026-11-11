import os
import time
from threading import Thread
from typing import Any, Callable, TypeVar

import rclpy
import yaml
from ament_index_python import get_package_share_directory
from beartype import BeartypeConf, BeartypeStrategy, beartype
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import PARAMETER_SEPARATOR_STRING, Parameter, parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.wait_for_message import wait_for_message

from bitbots_msgs.msg import ParameterBlackboard

# Create a new @nobeartype decorator disabling type-checking.
nobeartype = beartype(conf=BeartypeConf(strategy=BeartypeStrategy.O0))

# Topic on which the parameter_blackboard node publishes its complete state
GLOBAL_PARAMETER_BLACKBOARD_TOPIC = "global_parameter_blackboard"

# QoS for latched topics like the global parameter blackboard or the field maps, so late subscribers get the last message
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

MsgT = TypeVar("MsgT")


class RobotNotConfiguredError(Exception):
    """Exception raised when the robot is not configured properly."""

    def __init__(self, message):
        self.message = message
        super().__init__(message)


class MissingGlobalParameterError(RobotNotConfiguredError):
    """Exception raised when requested parameters are not set on the global parameter blackboard."""


def read_urdf(robot_name: str) -> str:
    urdf = os.popen(
        f"xacro {get_package_share_directory(f'{robot_name}_description')}"
        f"/urdf/robot.urdf use_fake_walk:=false sim_ns:=false"
    ).read()
    return urdf


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


def wait_for_latched_message(
    node: Node,
    msg_type: type[MsgT],
    topic: str,
    timeout_sec: float | None = None,
    warn_interval_sec: float = 5.0,
) -> MsgT:
    """
    Waits for a message on a latched (transient local) topic and periodically warns while waiting.

    The node is not spun, so this can be used during initialization before the node is added to an executor.
    Do not call it while an executor is spinning the node, because the executor might take the message first.

    :param node: Node used to create the temporary subscription
    :param msg_type: Message type of the topic
    :param topic: Name of the topic, relative names are resolved in the namespace of the node
    :param timeout_sec: Maximum time to wait, None waits until the message arrives or ROS shuts down
    :param warn_interval_sec: Interval in which a warning is logged while waiting
    :return: The received message
    :raises TimeoutError: If the timeout expires before a message is received
    :raises RuntimeError: If ROS shuts down while waiting
    """
    start_time = time.monotonic()
    while rclpy.ok():
        wait_time = warn_interval_sec
        if timeout_sec is not None:
            wait_time = min(wait_time, max(timeout_sec - (time.monotonic() - start_time), 0.0))
        success, msg = wait_for_message(msg_type, node, topic, qos_profile=LATCHED_QOS, time_to_wait=wait_time)
        if success:
            return msg

        waited = time.monotonic() - start_time
        if timeout_sec is not None and waited >= timeout_sec:
            raise TimeoutError(f"No message received on the latched topic '{topic}' within {waited:.1f}s")
        node.get_logger().warning(f"Still waiting for a message on the latched topic '{topic}' ({waited:.0f}s)")
    raise RuntimeError(f"ROS was shut down while waiting for a message on the latched topic '{topic}'")


def global_parameter_blackboard_to_dict(msg: ParameterBlackboard) -> dict[str, Any]:
    """
    Converts the state of the global parameter blackboard into a flat dict, which maps parameter names to values.
    """
    return {param.name: parameter_value_to_python(param.value) for param in msg.parameters}


def get_global_parameters(node: Node, parameter_names: list[str], timeout_sec: float | None = None) -> dict[str, Any]:
    """
    Gets parameters from the global parameter blackboard by waiting for its latched state topic.

    See :func:`wait_for_latched_message` for the restrictions regarding spinning nodes.

    :param node: Node used to create the temporary subscription
    :param parameter_names: Names of the requested parameters (e.g. 'field.size.x')
    :param timeout_sec: Maximum time to wait for the blackboard, None waits until it is available
    :return: Dict with the requested parameter names as keys and their values
    :raises MissingGlobalParameterError: If any of the requested parameters is not set on the blackboard
    """
    state = global_parameter_blackboard_to_dict(
        wait_for_latched_message(node, ParameterBlackboard, GLOBAL_PARAMETER_BLACKBOARD_TOPIC, timeout_sec)
    )
    missing = [name for name in parameter_names if name not in state]
    if missing:
        raise MissingGlobalParameterError(
            f"The parameters {missing} are not set on the global parameter blackboard. "
            "Check the configuration loaded by the parameter_blackboard node."
        )
    return {name: state[name] for name in parameter_names}


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
    ROS 2 does not provide an async sleep function, so we implement our own using a timer.
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


async def async_run_thread(func: Callable[[], Any]) -> None:
    """
    Allows the usage of blocking functions in an async context.

    Spawns a thread to run the function and returns a Future that will be set when the function is done.
    """
    future = Future()

    def thread_func():
        try:
            future.set_result(func())
        except Exception as e:
            future.set_exception(e)

    thread = Thread(target=thread_func)
    thread.start()
    await future
