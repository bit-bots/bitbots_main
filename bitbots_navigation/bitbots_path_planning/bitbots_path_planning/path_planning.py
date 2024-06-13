import rclpy
import soccer_vision_3d_msgs.msg as sv3dm
from bitbots_tf_buffer import Buffer
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty

from bitbots_msgs.msg import PoseWithCertaintyStamped
from bitbots_path_planning.controller import Controller
from bitbots_path_planning.map import Map
from bitbots_path_planning.path_planning_parameters import bitbots_path_planning as parameters
from bitbots_path_planning.planner import Planner


class PathPlanning(Node):
    """
    A minimal python path planning.
    """

    def __init__(self) -> None:
        super().__init__("bitbots_path_planning")
        self.param_listener = parameters.ParamListener(self)
        self.config = self.param_listener.get_params()

        # We need to create a tf buffer
        self.tf_buffer = Buffer(self, Duration(seconds=self.config.tf_buffer_duration))

        # Create submodules
        self.map = Map(node=self, buffer=self.tf_buffer)
        self.planner = Planner(node=self, buffer=self.tf_buffer, map=self.map)
        self.controller = Controller(node=self, buffer=self.tf_buffer)

        # Subscriber
        self.create_subscription(
            PoseWithCertaintyStamped,
            self.config.map.ball_update_topic,
            self.map.set_ball,
            5,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(
            sv3dm.RobotArray,
            self.config.map.robot_update_topic,
            self.map.set_robots,
            5,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, "goal_pose", self.planner.set_goal, 5, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.create_subscription(
            Empty,
            "pathfinding/cancel",
            lambda _: self.planner.cancel(),
            5,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.costmap_pub = self.create_publisher(OccupancyGrid, "costmap", 1)
        self.path_pub = self.create_publisher(Path, "path", 1)
        self.carrot_pub = self.create_publisher(PointStamped, "carrot", 1)

        # Timer that updates the path and command velocity at a given rate
        self.create_timer(
            1 / self.config.rate,
            self.step,
            clock=self.get_clock(),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def step(self) -> None:
        """
        Performs a single step of the path planning
        """
        if self.param_listener.is_old(self.config):
            self.param_listener.refresh_dynamic_parameters()
            self.config = self.param_listener.get_params()
        try:
            # Update the map with the latest ball and robot positions
            self.map.update()
            # Publish the costmap for visualization
            self.costmap_pub.publish(self.map.to_msg())

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

    # choose number of threads by number of callback_groups + 1 for simulation time
    ex = MultiThreadedExecutor(num_threads=7)
    ex.add_node(node)
    ex.spin()

    node.destroy_node()
    rclpy.shutdown()
