from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message

from bitbots_tf_listener.cpp_wrapper import TransformListener as TransformListener_


class TransformListener:
    def __init__(self, buffer, node, *args, **kwargs):
        self._impl = TransformListener_(node, self)
        self.buffer = buffer

    def set_transform(self, transform_str):
        transform = deserialize_message(transform_str, TransformStamped)
        self.buffer.set_transform(transform, "default_authority")

    def set_transform_static(self, transform_str):
        transform = deserialize_message(transform_str, TransformStamped)
        self.buffer.set_transform_static(transform, "default_authority")
