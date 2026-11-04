from ipaddress import ip_address
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from rclpy.node import Node

from bitbots_team_communication.communication import SocketCommunication


@pytest.mark.parametrize(("robot_id", "expected_port"), [(1, 4061), (2, 4062), (3, 4063), (4, 4064)])
def test_local_robot_skips_reserved_budget_port(robot_id: int, expected_port: int) -> None:
    parameters = {
        "target_ip": "127.0.0.1",
        "use_sim_time": True,
        "local_target_ports": [4000, 4001, 4002, 4003, 4004],
    }
    node = Node.__new__(Node)

    with patch(
        "bitbots_team_communication.communication.resolve_target_ip",
        return_value=(ip_address("127.0.0.1"), None),
    ):
        with patch.object(Node, "get_parameter", lambda self, name: SimpleNamespace(value=parameters[name])):
            communication = SocketCommunication(node, Mock(), team_id=6, robot_id=robot_id)

    assert communication.target_ports[0] == 4060
    assert communication.receive_port == expected_port
