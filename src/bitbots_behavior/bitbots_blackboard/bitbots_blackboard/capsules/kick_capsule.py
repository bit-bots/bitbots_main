import math
from enum import Flag

from geometry_msgs.msg import PoseStamped
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
    kick_direction_pub: Publisher

    def __init__(self, node, blackboard):
        super().__init__(node, blackboard)
        """
        :param blackboard: Global blackboard instance
        """
        self.walk_kick_pub = self._node.create_publisher(Bool, "/kick", 1)
        self.rl_kick_active_pub = self._node.create_publisher(Bool, "rl_kick_active", 1)
        self.kick_direction_pub = self._node.create_publisher(PoseStamped, "kick_direction", 1)

    def walk_kick(self, target: WalkKickTargets) -> None:
        """
        Kick the ball while walking
        :param target: Target for the walk kick (e.g. left or right foot)
        """
        self.walk_kick_pub.publish(Bool(data=target.value))

    def start_rl_kick(self, direction: float = 0.0) -> None:
        """
        Request the RL kick motion from the HCM.

        :param direction: Desired kick direction as a yaw angle (radians) relative to the robot
            (``base_footprint`` frame). 0.0 means straight ahead.
        """
        # Publish the direction first so the kick node has a valid target once it activates.
        self.kick_direction_pub.publish(self._kick_direction_msg(direction))
        self.rl_kick_active_pub.publish(Bool(data=True))

    def stop_rl_kick(self) -> None:
        self.rl_kick_active_pub.publish(Bool(data=False))

    def _kick_direction_msg(self, direction: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.orientation.z = math.sin(direction / 2.0)
        msg.pose.orientation.w = math.cos(direction / 2.0)
        return msg
