from typing import Optional

import tf2_ros as tf2
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

    def __init__(self, node, cache_time: Optional[Duration] = None, *args, **kwargs):
        if cache_time is not None:
            tf2.BufferCore.__init__(self, cache_time)
        else:
            tf2.BufferCore.__init__(self)
        tf2.BufferInterface.__init__(self)

        self._impl = CppBuffer(serialize_message(Duration.to_msg(cache_time)), node)

    def lookup_transform(
        self, target_frame: str, source_frame: str, time: Time, timeout: Optional[Duration] = None
    ) -> TransformStamped:
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
        self, target_frame: str, source_frame: str, time: Time, timeout: Optional[Duration] = None
    ) -> bool:
        # Handle timeout as None
        timeout = timeout or Duration()
        # Call cpp implementation
        return self._impl.can_transform(
            target_frame,
            source_frame,
            serialize_message(time if isinstance(time, TimeMsg) else Time.to_msg(time)),
            serialize_message(timeout if isinstance(timeout, DurationMsg) else Duration.to_msg(timeout)),
        )
