import rclpy
import soccer_vision_3d_msgs.msg as sv3dm
import tf2_ros as tf2
from bitbots_path_planning.controller import Controller
from bitbots_path_planning.map import Map
from bitbots_path_planning.planner import Planner
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty

from humanoid_league_msgs.msg import PoseWithCertaintyStamped


class PathPlanning(Node):

    def __init__(self) -> None:
        super().__init__('bitbots_path_planning')

        # Declare params
        self.declare_parameter('base_footprint_frame', 'base_footprint')
        self.declare_parameter('rate', 20.0)

        # We need to create a tf buffer
        self.tf_buffer = tf2.Buffer(
            cache_time=Duration(seconds=self.declare_parameter('tf_buffer_duration', 5.0).value))
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self)


        self.map = Map(node=self, buffer=self.tf_buffer)

        self.planner = Planner(node=self, buffer=self.tf_buffer, map=self.map)

        self.controller = Controller(node=self, buffer=self.tf_buffer)

        # Subscribe
        callback_group = ReentrantCallbackGroup()
        self.create_subscription(
            PoseWithCertaintyStamped,
            self.declare_parameter('map.ball_update_topic', 'ball_relative_filtered').value,
            self.map.set_ball,
            5,
            callback_group=callback_group)
        self.create_subscription(
            sv3dm.RobotArray,
            self.declare_parameter('map.robot_update_topic', 'robots_relative_filtered').value,
            self.map.set_robots,
            5,
            callback_group=callback_group)

        self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.planner.set_goal,
            5,
            callback_group=callback_group)
        self.create_subscription(
            Empty,
            'move_base/cancel',
            lambda _: self.planner.cancel(),
            5,
            callback_group=callback_group)

        # Publish cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.debug_costmap_pub = self.create_publisher(OccupancyGrid, 'costmap', 1)
        self.debug_path_pub = self.create_publisher(Path, 'path', 1)

        self.create_timer(1 / self.get_parameter('rate').value, self.step, clock=self.get_clock())

    def step(self):
        self.map.update()
        self.debug_costmap_pub.publish(self.map.to_msg())

        if self.planner.active():
            path = self.planner.step()
            self.debug_path_pub.publish(path)

            cmd_vel = self.controller.step(path)
            self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanning()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.spin()
    node.destroy_node()
    rclpy.shutdown()
