from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient

from bitbots_msgs.action import Kick


class KickCapsule():
    last_feedback = None  # type: Kick.Feedback
    last_feedback_received = None  # type: Time
    last_goal = None  # type: Kick.Goal
    last_goal_sent = None  # type: Time
    last_result = None  # type: Kick.Result
    last_result_received = None  # type: Time

    is_currently_kicking = False   # type: bool

    __connected = False  # type: bool
    __action_client = None  # type: actionlib.SimpleActionClient

    def __init__(self, blackboard):
        """
        :param blackboard: Global blackboard instance
        """
        self.__blackboard = blackboard
        self.connect()

    def connect(self):
        topic = self.__blackboard.config["dynamic_kick"]["topic"]
        self.__blackboard.nodeget_logger().info("Connecting {}.KickCapsule to bitbots_dynamic_kick ({})"
                      .format(str(self.__blackboard.__class__).split(".")[-1], topic))
        self.__action_client = ActionClient(self, Kick, topic)
        self.__connected = self.__action_client.wait_for_server(
            Duration(seconds=self.__blackboard.config["dynamic_kick"]["wait_time"]))

        if not self.__connected:
            self.__blackboard.nodeget_logger().error("No dynamic_kick server running on {}".format(topic))

    def kick(self, goal):
        """
        :param goal: Goal to kick to
        :type goal: KickGoal
        :raises RuntimeError: when not connected to dynamic_kick server
        """
        if not self.__connected:
            # try to connect again
            self.__connected = self.__action_client.wait_for_server(
                Duration(seconds=self.__blackboard.config["dynamic_kick"]["wait_time"]))
            if not self.__connected:
                raise RuntimeError("Not connected to any dynamic_kick server")

        self.__action_client.send_goal_async(goal, self.__done_cb, self.__active_cb, self.__feedback_cb)
        self.last_goal = goal
        self.last_goal_sent = self.__blackboard.node.get_clock().now()

    def __done_cb(self, state, result):
        self.last_result = Kick.Result(status=state, result=result)
        self.last_result_received = self.__blackboard.node.get_clock().now()
        self.is_currently_kicking = False

    def __feedback_cb(self, feedback):
        self.last_feedback = feedback
        self.last_feedback_received = self.__blackboard.node.get_clock().now()

    def __active_cb(self):
        self.is_currently_kicking = True
