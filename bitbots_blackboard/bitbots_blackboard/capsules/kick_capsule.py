import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from bitbots_msgs.msg import KickAction, KickFeedback, KickActionResult, KickGoal
from actionlib_msgs.msg import GoalStatus


class KickCapsule():
    last_feedback = None  # type: KickFeedback
    last_feedback_received = None  # type: rospy.Time
    last_goal = None  # type: KickGoal
    last_goal_sent = None  # type: rospy.Time
    last_result = None  # type: KickActionResult
    last_result_received = None  # type: rospy.Time

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
        self.get_logger().info("Connecting {}.KickCapsule to bitbots_dynamic_kick ({})"
                      .format(str(self.__blackboard.__class__).split(".")[-1], topic))
        self.__action_client = ActionClient(self, KickAction, topic)
        self.__connected = self.__action_client.wait_for_server(
            Duration(seconds=self.__blackboard.config["dynamic_kick"]["wait_time"]))

        if not self.__connected:
            self.get_logger().error("No dynamic_kick server running on {}".format(topic))

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
        self.last_goal_sent = self.get_clock().now()

    def __done_cb(self, state, result):
        self.last_result = KickActionResult(status=state, result=result)
        self.last_result_received = self.get_clock().now()
        self.is_currently_kicking = False

    def __feedback_cb(self, feedback):
        self.last_feedback = feedback
        self.last_feedback_received = self.get_clock().now()

    def __active_cb(self):
        self.is_currently_kicking = True
