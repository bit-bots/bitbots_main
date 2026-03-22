#!/usr/bin/env python3

import socket
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from udp_bridge.message_handler import MessageHandler


class UdpBridgeReceiver:
    def __init__(self, node: Node):
        self.node = node
        port: str = node.get_parameter("port").value
        self.node.get_logger().info(f"Initializing udp_bridge on port {port}")

        self.sock = socket.socket(type=socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(1)

        self.known_senders: list[str] = []
        self.publishers = {}

        encryption_key: str | None = None
        if node.has_parameter("encryption_key"):
            encryption_key = node.get_parameter("encryption_key").value

        self.message_handler = MessageHandler(encryption_key)

    def recv_message(self):
        """
        Receive a message from the network, process it and publish it into ROS
        """
        while rclpy.ok():
            try:
                # 65535 is the upper limit for the size because of network properties
                msg = self.sock.recv(65535)
                self.handle_message(msg)
            except socket.timeout:
                pass

    def handle_message(self, msg: bytes):
        """
        Handle a new message which came in from the socket
        """
        try:
            deserialized_msg = self.message_handler.decrypt_and_decode(msg)
            msg_type_name = deserialized_msg.get("msg_type_name")
            data = deserialize_message(deserialized_msg.get("data"), get_message(msg_type_name))
            topic: str = deserialized_msg.get("topic")
            hostname: str = deserialized_msg.get("hostname")
            latched: bool = deserialized_msg.get("latched")

            if hostname not in self.known_senders:
                self.known_senders.append(hostname)

            self.publish(topic, data, hostname, latched)
        except Exception as e:
            self.node.get_logger().error(f"Could not deserialize received message with error {e}")

    def publish(self, topic: str, msg, hostname: str, latched: bool):
        """
        Publish a message into ROS

        :param topic: The topic on which the message was sent on the originating host
        :param msg: The ROS message which was sent on the originating host
        :param hostname: The hostname of the originating host
        """

        # publish msg under host namespace
        namespaced_topic = hostname.replace("-", "_") + topic

        # create a publisher object if we don't have one already
        if namespaced_topic not in self.publishers.keys():
            self.node.get_logger().info(f"Publishing new topic {namespaced_topic}")
            self.publishers[namespaced_topic] = self.node.create_publisher(
                type(msg),
                namespaced_topic,
                qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL) if latched else 10,
            )

        self.publishers[namespaced_topic].publish(msg)


# @TODO: replace by usage of https://github.com/PickNikRobotics/generate_parameter_library
def validate_params(node: Node) -> bool:
    result = True

    if not node.has_parameter("port"):
        node.get_logger().fatal("parameter 'port' not found")
        result = False
    if not isinstance(node.get_parameter("port").value, int):
        node.get_logger().fatal("parameter 'port' is not an Integer")
        result = False

    return result


def run_spin_in_thread(node):
    # Necessary in ROS 2, or else we get stuck
    thread = Thread(target=rclpy.spin, args=[node], daemon=True)
    thread.start()


def main():
    rclpy.init()
    node = Node("udp_bridge_receiver", automatically_declare_parameters_from_overrides=True)

    if validate_params(node):
        # setup udp receiver
        receiver = UdpBridgeReceiver(node)
        run_spin_in_thread(node)
        receiver.recv_message()
