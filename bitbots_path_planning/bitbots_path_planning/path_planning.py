# Copyright (c) 2022 Hamburg Bit-Bots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from bitbots_path_planning.controller import Controller
from bitbots_path_planning.planner import Planner
from bitbots_path_planning.map import Map
from geometry_msgs.msg import PoseStamped
from humanoid_league_msgs.msg import PoseWithCertaintyStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Empty
import soccer_vision_3d_msgs.msg as sv3dm
from profilehooks import profile
import tf2_ros as tf2


class PathPlanning(Node):

    def __init__(self) -> None:
        super().__init__('bitbots_path_planning')
        # We need to create a tf buffer
        self.tf_buffer = tf2.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self)


        # Declare params
        self.declare_parameter('base_footprint_frame', 'base_footprint')
        self.declare_parameter('rate', 20)
        self.declare_parameter('map.planning_frame', 'map')
        self.declare_parameter('map.size.x', 11.0)
        self.declare_parameter('map.size.y', 8.0)
        self.declare_parameter('map.resolution', 20)

        self.map = Map(
            node=self,
            buffer=self.tf_buffer,
            size=(self.get_parameter('map.size.x').value, self.get_parameter('map.size.y').value),
            resolution=self.get_parameter('map.resolution').value,
            frame=self.get_parameter('map.planning_frame').value)

        self.planner = Planner(
            node=self,
            buffer=self.tf_buffer,
            map=self.map)

        self.controller = Controller(
            node=self,
            buffer=self.tf_buffer)

        # Subscribe
        self.create_subscription(PoseWithCertaintyStamped, 'ball_relative_filtered', self.map.set_ball, 5)
        self.create_subscription(sv3dm.RobotArray, 'robots_relative_filtered', self.map.set_robots, 5)

        self.create_subscription(PoseStamped, 'goal_pose', self.planner.set_goal, 5)
        self.create_subscription(Empty, 'move_base/cancel', lambda _: self.planner.cancel, 5)

        # Publish cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.debug_costmap_pub = self.create_publisher(OccupancyGrid, 'costmap', 1)
        self.debug_path_pub = self.create_publisher(Path, 'path', 1)

        self.create_timer(1 / self.get_parameter('rate').value, self.step, clock=self.get_clock())

    @profile
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
