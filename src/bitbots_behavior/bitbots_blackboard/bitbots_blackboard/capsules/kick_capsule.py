from enum import Flag

from rclpy.publisher import Publisher
from std_msgs.msg import Bool

from bitbots_blackboard.capsules import AbstractBlackboardCapsule


class KickCapsule(AbstractBlackboardCapsule):
    """Communicates with the  action server to kick the ball."""

    is_currently_kicking: bool = False

    class WalkKickTargets(Flag):
        """
        Define the target for the walk kick (e.g. left or right foot)
        """

        LEFT = False
        RIGHT = True

    walk_kick_pub: Publisher
    rl_kick_active_pub: Publisher

    def __init__(self, node, blackboard):
        super().__init__(node, blackboard)
        """
        :param blackboard: Global blackboard instance
        """
        self.walk_kick_pub = self._node.create_publisher(Bool, "/kick", 1)
        self.rl_kick_active_pub = self._node.create_publisher(Bool, "rl_kick_active", 1)

    def walk_kick(self, target: WalkKickTargets) -> None:
        """
        Kick the ball while walking
        :param target: Target for the walk kick (e.g. left or right foot)
        """
        self.walk_kick_pub.publish(Bool(data=target.value))

    def start_rl_kick(self):
        self.rl_kick_active_pub.publish(Bool(data=True))

    def stop_rl_kick(self):
        self.rl_kick_active_pub.publish(Bool(data=False))
