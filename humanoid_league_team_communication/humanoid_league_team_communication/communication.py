import socket

from rclpy.node import Node


class SocketCommunication:

    def __init__(self, node: Node, logger, team_id, robot_id):
        self.logger = logger

        self.socket: socket.socket = None
        self.target_host = node.get_parameter('target_host').get_parameter_value().string_value

        if self.target_host == '127.0.0.1':
            # local mode, bind to port depending on bot id and team id
            local_target_ports = node.get_parameter('local_target_ports').get_parameter_value().integer_array_value
            self.target_ports = [port + 10 * team_id for port in local_target_ports]
            self.receive_port = self.target_ports[robot_id - 1]
        else:
            target_port = node.get_parameter('target_port').get_parameter_value().integer_value
            receive_port = node.get_parameter('receive_port').get_parameter_value().integer_value
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
        sock.bind(('0.0.0.0', self.receive_port))
        return sock

    def close_connection(self):
        if self.is_setup():
            self.socket.close()
            self.logger.info("Connection closed.")

    def receive_message(self) -> bytes:
        return self.socket.recv(1024)

    def send_message(self, message):
        for port in self.target_ports:
            self.logger.debug(f'Sending message to {port} on {self.target_host}')
            self.socket.sendto(message, (self.target_host, port))
