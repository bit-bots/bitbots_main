from enum import Flag

from rclpy.publisher import Duration, Publisher
from std_msgs.msg import Bool
from rclpy.action import ActionClient
import math
from bitbots_msgs.action import Kick
from bitbots_blackboard.capsules import AbstractBlackboardCapsule
from geometry_msgs.msg import Twist, Vector3

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
        self._timeout = blackboard.config["rl_kick"]["timeout"]
        self._post_kick_timeout = blackboard.config["rl_kick"]["post_kick_timeout"]
        self._walk_delay = blackboard.config["rl_kick"]["walk_delay"]

        self._rl_kick_client = ActionClient(self._node, Kick, "rl_kick")
        self._is_currently_kicking = False

    def walk_kick(self, target: WalkKickTargets) -> None:
        """
        Kick the ball while walking
        :param target: Target for the walk kick (e.g. left or right foot)
        """
        self.walk_kick_pub.publish(Bool(data=target.value))

    def start_rl_kick(self, direction_deg_map, strength):
        if not self._rl_kick_client.wait_for_server(timeout_sec=1.0):
            self._node.get_logger().error(
                "RL Kick Action Server not running!"
            )
            return False

        direction_rad_map = math.radians(direction_deg_map)
        _, _, robot_theta = self._blackboard.world_model.get_current_position()
        direction_rad_robot = direction_rad_map - robot_theta
        goal = Kick.Goal()
        goal.x = math.cos(direction_rad_robot)
        goal.y = math.sin(direction_rad_robot)
        goal.strength = strength
        goal.timeout = self._timeout
        self._rl_kick_client.send_goal_async(goal).add_done_callback(
            lambda future: future.result().get_result_async().add_done_callback(lambda _: self.__done_cb())
        )
        self.is_currently_kicking = True
        return True

    def __done_cb(self):
        self.is_currently_kicking = False

    def stop_rl_kick(self):
        pass
