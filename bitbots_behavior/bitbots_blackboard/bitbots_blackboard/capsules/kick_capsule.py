from enum import Flag
from typing import Optional

from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.publisher import Publisher
from rclpy.time import Time
from std_msgs.msg import Bool

from bitbots_blackboard.capsules import AbstractBlackboardCapsule
from bitbots_msgs.action import Kick


class KickCapsule(AbstractBlackboardCapsule):
    """Communicates with the dynamic_kick action server to kick the ball."""

    last_feedback: Optional[Kick.Feedback] = None
    last_feedback_received: Optional[Time] = None
    last_goal: Optional[Kick.Goal] = None
    last_goal_sent: Optional[Time] = None

    is_currently_kicking: bool = False

    __connected: bool = False
    __action_client: Optional[ActionClient] = None

    class WalkKickTargets(Flag):
        """
        Define the target for the walk kick (e.g. left or right foot)
        """

        LEFT = False
        RIGHT = True

    walk_kick_pub: Publisher

    def __init__(self, node, blackboard):
        super().__init__(node, blackboard)
        """
        :param blackboard: Global blackboard instance
        """
        self.walk_kick_pub = self._node.create_publisher(Bool, "/kick", 1)
        # self.connect_dynamic_kick()  Do not connect if dynamic_kick is disabled

    def walk_kick(self, target: WalkKickTargets) -> None:
        """
        Kick the ball while walking
        :param target: Target for the walk kick (e.g. left or right foot)
        """
        self.walk_kick_pub.publish(Bool(data=target.value))

    def connect_dynamic_kick(self) -> None:
        topic = self._blackboard.config["dynamic_kick"]["topic"]
        self.__action_client = ActionClient(self._node, Kick, topic, callback_group=ReentrantCallbackGroup())
        self.__connected = self.__action_client.wait_for_server(self._blackboard.config["dynamic_kick"]["wait_time"])

        if not self.__connected:
            self._node.get_logger().error(f"No dynamic_kick server running on {topic}")

    def dynamic_kick(self, goal: Kick.Goal) -> None:
        """
        :param goal: Goal to kick to
        :type goal: KickGoal
        :raises RuntimeError: when not connected to dynamic_kick server
        """
        raise NotImplementedError("The dynamic_kick is disabled currently")
        if not self.__connected:
            # try to connect again
            self.__connected = self.__action_client.wait_for_server(
                Duration(seconds=self._blackboard.config["dynamic_kick"]["wait_time"])
            )
            if not self.__connected:
                raise RuntimeError("Not connected to any dynamic_kick server")

        self.is_currently_kicking = True
        self.__action_client.send_goal_async(goal, self.__feedback_cb).add_done_callback(
            lambda future: future.result().get_result_async().add_done_callback(lambda _: self.__done_cb())
        )

        self.last_goal = goal
        self.last_goal_sent = self._node.get_clock().now()

    def __feedback_cb(self, feedback):
        self.last_feedback: Kick.Feedback = feedback.feedback
        self.last_feedback_received = self._node.get_clock().now()

    def __done_cb(self):
        self.is_currently_kicking = False
