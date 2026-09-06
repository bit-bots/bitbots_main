import math
from typing import Optional

import tf2_ros
from bitbots_tf_buffer import Buffer
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time


def wrap_to_pi(angle: float) -> float:
    """Wraps an angle into (-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_in_frame(node: Node, frame: str, child_frame: str, timeout_s: float) -> Optional[float]:
    """Returns the yaw of a frame relative to another one, or None if it is unavailable"""
    buffer = shared_tf_buffer(node)
    try:
        transform = buffer.lookup_transform(frame, child_frame, Time(), Duration(seconds=timeout_s))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        node.get_logger().warning(
            f"Could not look up the yaw of '{child_frame}' in '{frame}': {e}", throttle_duration_sec=1.0
        )
        return None
    q = transform.transform.rotation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def shared_tf_buffer(node: Node) -> Buffer:
    """Returns the transform buffer of a node, creating it on first use.

    Every handler that needs a transform would otherwise start its own listener,
    which subscribes to the whole transform tree once more.
    """
    buffer = getattr(node, "_shared_tf_buffer", None)
    if buffer is None:
        buffer = Buffer(node=node)
        node._shared_tf_buffer = buffer  # type: ignore[attr-defined]
    return buffer
