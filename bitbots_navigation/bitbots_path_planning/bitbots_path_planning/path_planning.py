import rclpy
import soccer_vision_3d_msgs.msg as sv3dm
from bitbots_tf_buffer import Buffer
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from std_msgs.msg import Bool, Empty
from visualization_msgs.msg import MarkerArray
from rclpy.experimental.events_executor import EventsExecutor

from bitbots_path_planning import NodeWithConfig
from bitbots_path_planning.controller import Controller
from bitbots_path_planning.planner import VisibilityPlanner


class PathPlanning(NodeWithConfig):
    """
    A minimal python path planning.
    """

    def __init__(self) -> None:
        super().__init__("bitbots_path_planning")

        # We need to create a tf buffer
        self.tf_buffer = Buffer(Duration(seconds=self.config.tf_buffer_duration), self)

        self.planner = VisibilityPlanner(node=self, buffer=self.tf_buffer)
        self.controller = Controller(node=self, buffer=self.tf_buffer)

        # Subscriber
        self.create_subscription(PoseWithCovarianceStamped, self.config.map.ball_update_topic, self.planner.set_ball, 5)
        self.create_subscription(sv3dm.RobotArray, self.config.map.robot_update_topic, self.planner.set_robots, 5)
        self.goal_sub = self.create_subscription(PoseStamped, "goal_pose", self.planner.set_goal, 5)
        self.create_subscription(Empty, "pathfinding/cancel", lambda _: self.planner.cancel_goal(), 5)
        self.create_subscription(
            Bool,
            "ball_obstacle_active",
            lambda msg: self.planner.avoid_ball(msg.data),  # type: ignore
            5,
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.path_pub = self.create_publisher(Path, "path", 1)
        self.carrot_pub = self.create_publisher(PointStamped, "carrot", 1)
        self.graph_pub = self.create_publisher(MarkerArray, "visibility_graph", 1)

        # Timer that updates the path and command velocity at a given rate
        self.create_timer(1 / self.config.rate, self.step, clock=self.get_clock())

    def step(self) -> None:
        """
        Performs a single step of the path planning
        """
        if self.param_listener.is_old(self.config):
            self.param_listener.refresh_dynamic_parameters()
            self.config = self.param_listener.get_params()
        try:
            if self.planner.active():
                # Calculate the path to the goal pose considering the current map
                path = self.planner.step()
                # Publish the path for visualization
                self.path_pub.publish(path)
                # Calculate the command velocity to follow the given path
                cmd_vel, carrot_point = self.controller.step(path)
                # Publish the walk command to control the robot
                self.cmd_vel_pub.publish(cmd_vel)
                # Publish the carrot point for visualization
                self.carrot_pub.publish(carrot_point)
        except Exception as e:
            self.get_logger().error(f"Caught exception during calculation of path planning step: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanning()

    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Path planning node stopped by user")

    node.destroy_node()
