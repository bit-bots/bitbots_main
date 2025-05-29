from typing import TYPE_CHECKING
from functools import wraps

from bitbots_utils.utils import nobeartype
from rclpy.node import Node

if TYPE_CHECKING:
    from bitbots_blackboard.body_blackboard import BodyBlackboard


def cached_capsule_function(method):
    """Decorator to cache the result of a method."""
    cache_key = method.__name__

    @wraps(method)
    def wrapper(self):
        if cache_key not in self._cache:
            self._cache[cache_key] = method(self)
        return self._cache[cache_key]

    return wrapper

class AbstractBlackboardCapsule:
    """Abstract class for blackboard capsules."""

    @nobeartype
    def __init__(self, node: Node, blackboard: "BodyBlackboard"):
        self._node = node
        self._blackboard = blackboard

        self._cache: dict[str, object] = {}

    def clear_cache(self):
        """Clear the cache of this capsule."""
        self._cache.clear()

