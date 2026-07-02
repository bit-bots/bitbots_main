import rclpy
from game_controller_hsl_interfaces.msg import PlayerStateResponse
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node

from bitbots_msgs.msg import RobotControlState


class PlayerStateAggregator(Node):
    """Aggregate native robot state into the GameController response interface."""

    FALLEN_STATES = {
        RobotControlState.FALLING,
        RobotControlState.FALLEN,
        RobotControlState.GETTING_UP,
    }

    def __init__(self) -> None:
        super().__init__("player_state_aggregator")

        robot_state_topic = self.declare_parameter("robot_state_topic", "robot_state").value
        pose_topic = self.declare_parameter("pose_topic", "pose_with_covariance").value
        ball_topic = self.declare_parameter("ball_topic", "ball_position_relative_filtered").value
        response_topic = self.declare_parameter("response_topic", "player_state_response").value
        publish_rate = self.declare_parameter("publish_rate", 1.0).value

        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")

        self._fallen = False
        self._pose: PoseStamped | None = None
        self._relative_ball: PointStamped | None = None

        self._publisher = self.create_publisher(
            PlayerStateResponse, response_topic, 1, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.create_subscription(
            RobotControlState,
            robot_state_topic,
            self._robot_state_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self._pose_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            ball_topic,
            self._ball_callback,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_timer(1.0 / publish_rate, self._publish, callback_group=MutuallyExclusiveCallbackGroup())

    def _robot_state_callback(self, msg: RobotControlState) -> None:
        self._fallen = msg.state in self.FALLEN_STATES

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose = PoseStamped(header=msg.header, pose=msg.pose.pose)

    def _ball_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._relative_ball = PointStamped(header=msg.header, point=msg.pose.pose.position)

    def build_response(self) -> PlayerStateResponse:
        """Build a consistent snapshot from the most recently received data."""
        response = PlayerStateResponse()
        response.header.stamp = self.get_clock().now().to_msg()
        response.fallen = self._fallen

        if self._pose is not None:
            response.pose = self._pose

        if self._relative_ball is not None:
            response.ball = self._relative_ball

        return response

    def _publish(self) -> None:
        self._publisher.publish(self.build_response())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlayerStateAggregator()

    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
