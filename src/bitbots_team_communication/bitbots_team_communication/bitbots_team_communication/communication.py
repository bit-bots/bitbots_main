import socket
import struct
import threading
from abc import ABC, abstractmethod
from typing import Callable, Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8MultiArray

from bitbots_team_communication.network import resolve_target_ip


class CommunicationBackend(ABC):
    """Transport used to exchange serialized team communication messages with other robots."""

    @abstractmethod
    def establish_connection(self) -> None: ...

    @abstractmethod
    def is_setup(self) -> bool: ...

    @abstractmethod
    def send_message(self, message: bytes) -> None: ...

    @abstractmethod
    def start_receiving(self, callback: Callable[[bytes], None]) -> None:
        """Start delivering incoming messages to `callback`, called with the raw bytes of each message."""

    @abstractmethod
    def close_connection(self) -> None: ...


class SocketCommunication(CommunicationBackend):
    def __init__(self, node: Node, logger, team_id, robot_id):
        self.logger = logger

        self.buffer_size: int = 1024
        self.socket: socket.socket | None = None
        configured_target_ip: str = node.get_parameter("target_ip").value
        self.target_ip, wifi_interface = resolve_target_ip(configured_target_ip)
        if wifi_interface is not None:
            self.logger.info(
                f"Using Wi-Fi broadcast address {self.target_ip} from interface {wifi_interface} for team communication"
            )

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

        self._running = False
        self._receive_thread: Optional[threading.Thread] = None

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
        self._running = False
        if self.is_setup():
            self.socket.close()  # type: ignore[union-attr]
            self.logger.info("Connection closed.")

    def start_receiving(self, callback: Callable[[bytes], None]) -> None:
        self._running = True

        def loop():
            while self._running and rclpy.ok():
                try:
                    message = self.receive_message()
                except (struct.error, socket.timeout):
                    continue
                if message:
                    callback(message)

        self._receive_thread = threading.Thread(target=loop, daemon=True)
        self._receive_thread.start()

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


class RosCommunication(CommunicationBackend):
    """Exchanges team communication messages as serialized binary blobs over a ROS topic.

    Used instead of `SocketCommunication` in simulation, where each robot runs in its own ROS
    domain ID (so a fixed UDP port can't be shared) and a `domain_bridge` instance per robot
    bridges `ros_topic` bidirectionally between that robot's domain and a shared hub domain,
    emulating the UDP broadcast used between real robots. As with UDP broadcast, a robot also
    receives its own messages back; these are filtered out downstream by player/team id.
    """

    def __init__(self, node: Node, logger):
        self.logger = logger
        self.node = node

        topic: str = node.get_parameter("ros_topic").value
        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = node.create_publisher(UInt8MultiArray, topic, self.qos)
        self.topic = topic

    def establish_connection(self) -> None:
        pass

    def is_setup(self) -> bool:
        return True

    def send_message(self, message: bytes) -> None:
        self.publisher.publish(UInt8MultiArray(data=list(message)))

    def start_receiving(self, callback: Callable[[bytes], None]) -> None:
        def handle_ros_message(msg: UInt8MultiArray) -> None:
            callback(bytes(msg.data))

        self.node.create_subscription(
            UInt8MultiArray,
            self.topic,
            handle_ros_message,
            qos_profile=self.qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def close_connection(self) -> None:
        pass
