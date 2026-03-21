from typing import Optional

import tf2_ros as tf2

from rclpy.node import Node
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time

from bitbots_tf_buffer.cpp_wrapper import Buffer as CppBuffer


class Buffer(tf2.BufferCore, tf2.BufferInterface):
    """
    Buffer class that wraps the C++ implementation of the tf2 buffer and listener.
    It spawns a new node with the suffix "_tf" to handle the C++ side of the ROS communication.
    """

    def __init__(self, cache_time: Optional[Duration] = None, node: Optional[Node] = None):
        if cache_time is None:
            cache_time = Duration(seconds=10.0)

        tf2.BufferCore.__init__(self, cache_time)
        tf2.BufferInterface.__init__(self)

        self.cache_time = cache_time
        self._impl: Optional[CppBuffer] = None

        # If a node is provided, we can set the node directly
        if node is not None:
            self.set_node(node)
            
    def set_node(self, node: Node):
        """
        This API is used instead of the constructor to set the node.
        This way we can have a dummy TransformListener and therefore
        keep compatibility with the official implementation.
        """
        self._impl = CppBuffer(serialize_message(Duration.to_msg(self.cache_time)), node)

    def lookup_transform(
        self, target_frame: str, source_frame: str, time: Time | TimeMsg, timeout: Optional[Duration | DurationMsg] = None
    ) -> TransformStamped:
        assert self._impl is not None, "Buffer has not been initialized with a node. You either need to pass a node to the constructor or have a TransformListener set up."
        # Handle timeout as None
        timeout = timeout or Duration()
        # Call cpp implementation
        transform_str = self._impl.lookup_transform(
            target_frame,
            source_frame,
            serialize_message(time if isinstance(time, TimeMsg) else Time.to_msg(time)),
            serialize_message(timeout if isinstance(timeout, DurationMsg) else Duration.to_msg(timeout)),
        )
        return deserialize_message(transform_str, TransformStamped)

    def can_transform(
        self, target_frame: str, source_frame: str, time: Time | TimeMsg, timeout: Optional[Duration | DurationMsg] = None
    ) -> bool:
        assert self._impl is not None, "Buffer has not been initialized with a node. You either need to pass a node to the constructor or have a TransformListener set up."
        # Handle timeout as None
        timeout = timeout or Duration()
        # Call cpp implementation
        return self._impl.can_transform(
            target_frame,
            source_frame,
            serialize_message(time if isinstance(time, TimeMsg) else Time.to_msg(time)),
            serialize_message(timeout if isinstance(timeout, DurationMsg) else Duration.to_msg(timeout)),
        )
    
class TransformListener:
    """
    A dummy TransformListener that just sets the node into the C++ Buffer.
    This is done for compatibility with the original API implementation.
    """
    def __init__(
        self,
        buffer: Buffer,
        node: Node,
        *ignored_args,
        **ignored_kwargs
    ) -> None:
        buffer.set_node(node)
