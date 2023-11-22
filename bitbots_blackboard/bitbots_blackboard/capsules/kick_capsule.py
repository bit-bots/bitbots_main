"""
KickCapsule
^^^^^^^^^^^^^^^^

Communicates with the dynamic_kick.
"""
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from bitbots_blackboard.blackboard import BodyBlackboard

from bitbots_msgs.action import Kick
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time


class KickCapsule:
    __blackboard: "BodyBlackboard"

    last_feedback: Optional[Kick.Feedback] = None
    last_feedback_received: Optional[Time] = None
    last_goal: Optional[Kick.Goal] = None
    last_goal_sent: Optional[Time] = None

    is_currently_kicking: bool = False

    __connected: bool = False
    __action_client: ActionClient = None

    def __init__(self, blackboard):
        """
        :param blackboard: Global blackboard instance
        """
        self.__blackboard = blackboard
        self.connect()

    def connect(self):
        topic = self.__blackboard.config["dynamic_kick"]["topic"]
        self.__blackboard.node.get_logger().info(
            "Connecting {}.KickCapsule to bitbots_dynamic_kick ({})".format(
                str(self.__blackboard.__class__).split(".")[-1], topic
            )
        )
        self.__action_client = ActionClient(
            self.__blackboard.node, Kick, topic, callback_group=ReentrantCallbackGroup()
        )
        self.__connected = self.__action_client.wait_for_server(self.__blackboard.config["dynamic_kick"]["wait_time"])

        if not self.__connected:
            self.__blackboard.node.get_logger().error(f"No dynamic_kick server running on {topic}")

    def kick(self, goal: Kick.Goal):
        """
        :param goal: Goal to kick to
        :type goal: KickGoal
        :raises RuntimeError: when not connected to dynamic_kick server
        """
        if not self.__connected:
            # try to connect again
            self.__connected = self.__action_client.wait_for_server(
                Duration(seconds=self.__blackboard.config["dynamic_kick"]["wait_time"])
            )
            if not self.__connected:
                raise RuntimeError("Not connected to any dynamic_kick server")

        self.is_currently_kicking = True
        self.__action_client.send_goal_async(goal, self.__feedback_cb).add_done_callback(
            lambda future: future.result().get_result_async().add_done_callback(lambda _: self.__done_cb())
        )

        self.last_goal = goal
        self.last_goal_sent = self.__blackboard.node.get_clock().now()

    def __feedback_cb(self, feedback):
        self.last_feedback: Kick.Feedback = feedback.feedback
        self.last_feedback_received = self.__blackboard.node.get_clock().now()

    def __done_cb(self):
        self.is_currently_kicking = False
