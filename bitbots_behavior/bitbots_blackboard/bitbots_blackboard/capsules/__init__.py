from typing import TYPE_CHECKING, Optional

from rclpy.node import Node

if TYPE_CHECKING:
    from bitbots_blackboard.body_blackboard import BodyBlackboard


class AbstractBlackboardCapsule:
    """Abstract class for blackboard capsules."""

    def __init__(self, node: Node, blackboard: Optional["BodyBlackboard"] = None):
        self._node = node
        self._blackboard = blackboard
