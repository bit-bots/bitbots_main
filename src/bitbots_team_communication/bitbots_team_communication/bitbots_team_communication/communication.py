import socket
from ipaddress import IPv4Address
from typing import Optional

from rclpy.node import Node


class SocketCommunication:
    def __init__(self, node: Node, logger, team_id, robot_id):
        self.logger = logger

        self.buffer_size: int = 1024
        self.socket: Optional[socket.socket] = None
        self.target_ip: IPv4Address = IPv4Address(node.get_parameter("target_ip").value)

        if self.target_ip.is_loopback:
            # local mode on loopback device, bind to port depending on bot id and team id
            local_target_ports: list[int] = node.get_parameter("local_target_ports").value
            self.target_ports = [port + 10 * team_id for port in local_target_ports]
            self.receive_port = self.target_ports[robot_id - 1]
        else:
            target_port: int = node.get_parameter("target_port").value
            receive_port: int = node.get_parameter("receive_port").value
            self.target_ports = [target_port]
            self.receive_port = receive_port

    def __del__(self):
        self.close_connection()

    def is_setup(self) -> bool:
        return self.socket is not None

    def establish_connection(self):
        if not self.is_setup():
            self.socket = self.get_connection()

    def get_connection(self) -> socket.socket:
        self.logger.info(f"Binding to port {self.receive_port} to receive messages")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(("0.0.0.0", self.receive_port))
        return sock

    def close_connection(self):
        if self.is_setup():
            self.socket.close()  # type: ignore[union-attr]
            self.logger.info("Connection closed.")

    def receive_message(self) -> Optional[bytes]:
        self.assert_is_setup()
        msg, _, flags, _ = self.socket.recvmsg(self.buffer_size)  # type: ignore[union-attr]
        is_message_truncated = flags & socket.MSG_TRUNC
        if is_message_truncated:
            self.handle_truncated_message()
            return None
        else:
            return msg

    def handle_truncated_message(self):
        self.logger.warn(
            f"recvmsg flag {socket.MSG_TRUNC} signaling a packet larger than buffer size {self.buffer_size}"
        )
        self.buffer_size *= 2
        self.logger.info(f"doubled buffer size to {self.buffer_size}")

    def send_message(self, message):
        self.assert_is_setup()
        for port in self.target_ports:
            self.logger.debug(f"Sending message to {port} on {self.target_ip}")
            self.socket.sendto(message, (str(self.target_ip), port))  # type: ignore[union-attr]

    def assert_is_setup(self):
        assert self.is_setup(), "Socket is not yet initialized"
