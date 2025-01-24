from typing import TYPE_CHECKING

from bitbots_utils.utils import nobeartype
from rclpy.node import Node

if TYPE_CHECKING:
    from bitbots_blackboard.body_blackboard import BodyBlackboard


class AbstractBlackboardCapsule:
    """Abstract class for blackboard capsules."""

    @nobeartype
    def __init__(self, node: Node, blackboard: "BodyBlackboard"):
        self._node = node
        self._blackboard = blackboard
