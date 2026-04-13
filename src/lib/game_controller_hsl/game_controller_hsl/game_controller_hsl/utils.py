from typing import Any

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python


def get_parameters_from_other_node(own_node: Node,
                                   other_node_name: str,
                                   parameter_names: list[str],
                                   service_timeout_sec: float = 20.0) -> dict[str, Any]:
    """
    Used to receive parameters from other running nodes.
    Returns a dict with requested parameter name as dict key and parameter value as dict value.
    
    From bitbots_utils (https://github.com/bit-bots/bitbots_misc)
    """
    client = own_node.create_client(GetParameters, f'{other_node_name}/get_parameters')
    ready = client.wait_for_service(timeout_sec=service_timeout_sec)
    if not ready:
        raise RuntimeError(f'Wait for {other_node_name} parameter service timed out')
    request = GetParameters.Request()
    request.names = parameter_names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(own_node, future)
    response = future.result()

    results = {}  # Received parameter
    for i, param in enumerate(parameter_names):
        results[param] = parameter_value_to_python(response.values[i])
    return results