#!/usr/bin/env python3

import socket
from queue import Empty, Full, Queue

import rclpy
from bitbots_utils.utils import RobotNotConfiguredError, get_parameters_from_other_node
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import serialize_message
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from rosidl_runtime_py.utilities import get_message

from udp_bridge.message_handler import MessageHandler

HOSTNAME = socket.gethostname()


class AutoSubscriber:
    """
    A class which automatically subscribes to a topic as soon as it becomes available and buffers received messages
    in a queue.
    """

    def __init__(self, topic: str, queue_size: int, message_handler: MessageHandler, node: Node):
        """
        :param topic: Topic to subscribe to
        :type topic: str
        :param queue_size: How many received messages should be buffered
        :type queue_size: int
        """
        self.topic: str = topic
        self.queue: Queue = Queue(queue_size)
        self.message_handler: MessageHandler = message_handler
        self.node: Node = node
        self.timer: Timer | None = None
        self.msg_type_name: str = None

        self.__subscriber: Subscription | None = None
        self.__latched_subscriber: Subscription | None = None
        self.__subscribe()

    def __subscribe(self, backoff=1.0):
        """
        Try to subscribe to the set topic
        :param backoff: How long to wait until another try (capped at 30s)
        """
        if backoff > 30:
            backoff = 30

        if self.timer:
            self.timer.cancel()

        data_class = None
        for topic, msg_type_names in self.node.get_topic_names_and_types():
            if topic == self.topic:
                self.msg_type_name = msg_type_names[0]
                data_class = get_message(self.msg_type_name)
                # topic is known
                self.node.get_logger().debug(f"Want to subscribe to topic {self.topic}")
                # find out if topic is latched / transient local
                publisher_infos = self.node.get_publishers_info_by_topic(topic)
                latched = any(
                    info.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for info in publisher_infos
                )
                self.__subscriber = self.node.create_subscription(data_class, self.topic, self.__message_callback, 1)
                if latched:
                    self.__latched_subscriber = self.node.create_subscription(
                        data_class,
                        self.topic,
                        lambda msg: self.__message_callback(msg, latched=True),
                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                    )
                self.node.get_logger().debug(f"Subscribed to topic {self.topic}")
                return

        # topic is not yet known
        if backoff > 10:
            logging_severity = LoggingSeverity.WARN
        else:
            logging_severity = LoggingSeverity.DEBUG
        self.node.get_logger().log(
            f"Topic {self.topic} is not yet known. Retrying in {backoff} seconds", logging_severity
        )
        self.timer = self.node.create_timer(backoff, lambda: self.__subscribe(backoff * 1.2))

    def __message_callback(self, data, latched=False):
        encrypted_msg = self.message_handler.encrypt_and_encode(
            {
                "data": serialize_message(data),
                "topic": self.topic,
                "msg_type_name": self.msg_type_name,
                "hostname": HOSTNAME,
                "latched": latched,
            }
        )

        try:
            self.queue.put(encrypted_msg, block=True, timeout=1)
        except Full:
            self.node.get_logger().warn(f"Could not enqueue new message of topic {self.topic}. Queue full.")

        # for latched messages, republish them every ten seconds because we cannot latch on the other side
        if latched:
            if self.timer:
                self.timer.cancel()
            self.timer = self.node.create_timer(10.0, lambda: self.__message_callback(data, latched=True))


# @TODO: replace by usage of https://github.com/PickNikRobotics/generate_parameter_library
def validate_params(node: Node) -> bool:
    result = True

    if not node.has_parameter("port"):
        node.get_logger().fatal("parameter 'port' not found")
        result = False
    if not isinstance(node.get_parameter("port").value, int):
        node.get_logger().fatal("parameter 'port' is not an Integer")
        result = False

    if not node.has_parameter("topics"):
        node.get_logger().fatal("parameter 'port' not found")
        result = False
    if not isinstance(node.get_parameter("topics").value, list):
        node.get_logger().fatal("parameter 'topics' is not a list")
        result = False
    if len(node.get_parameter("topics").value) == 0:
        node.get_logger().warn("parameter 'topics' is an empty list")

    if not node.has_parameter("sender_queue_max_size"):
        node.get_logger().fatal("parameter 'sender_queue_max_size' not found")
        result = False
    if not isinstance(node.get_parameter("sender_queue_max_size").value, int):
        node.get_logger().fatal("parameter 'sender_queue_max_size' is not an Integer")
        result = False

    if not node.has_parameter("send_frequency"):
        node.get_logger().fatal("parameter 'send_frequency' not found")
        result = False
    if not isinstance(node.get_parameter("send_frequency").value, float) and not isinstance(
        node.get_parameter("send_frequency").value, int
    ):
        node.get_logger().fatal("parameter 'send_frequency' is not an Integer or Float")
        result = False

    return result


class UdpBridgeSender:
    def __init__(self, node: Node):
        self.node = node
        self.freq: float = node.get_parameter("send_frequency").value

        target_ip_parameter_name: str = "monitoring_host_ip"
        self.params_blackboard = get_parameters_from_other_node(
            node, "parameter_blackboard", [target_ip_parameter_name]
        )
        if any(param_val is None for param_val in self.params_blackboard.values()):
            error_text = """
The robot is not configured properly or the parameter_blackboard is not found.
It is likely that the robot was not configured when you syncronised your clean code onto the robot.
Run the deploy_robots script with the -c option to configure the robot and set parameters like the robot id and its role."""
            self._node.get_logger().fatal(error_text)
            raise RobotNotConfiguredError(error_text)
        self.target: str = self.params_blackboard[target_ip_parameter_name]
        self.port: int = node.get_parameter("port").value
        self.sock = self.setup_udp_socket()

        topics: list[str] = node.get_parameter("topics").value
        max_queue_size: int = node.get_parameter("sender_queue_max_size").value
        message_handler = self.setup_message_handler()
        self.subscribers: list[AutoSubscriber] = list(
            map(lambda topic: AutoSubscriber(topic, max_queue_size, message_handler, node), topics)
        )

    def setup_udp_socket(self) -> socket.socket:
        sock = socket.socket(type=socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        return sock

    def setup_message_handler(self) -> MessageHandler:
        encryption_key: str | None = None
        if self.node.has_parameter("encryption_key"):
            encryption_key = self.node.get_parameter("encryption_key").value

        return MessageHandler(encryption_key)

    def send_messages_in_queue(self):
        for subscriber in self.subscribers:
            try:
                data = subscriber.queue.get_nowait()

                try:
                    self.sock.sendto(data, (self.target, self.port))
                except Exception as e:
                    self.node.get_logger().error(
                        f"Could not send data of topic {subscriber.topic} to {self.target} with error {str(e)}"
                    )

            except Empty:
                pass


def main():
    rclpy.init()
    node = Node("udp_bridge_sender", automatically_declare_parameters_from_overrides=True)

    if validate_params(node):
        sender = UdpBridgeSender(node)

        exec = SingleThreadedExecutor()
        exec.add_node(node)
        node.create_timer((1 / sender.freq), sender.send_messages_in_queue)
        exec.spin()

        node.destroy_node()
        rclpy.shutdown()
