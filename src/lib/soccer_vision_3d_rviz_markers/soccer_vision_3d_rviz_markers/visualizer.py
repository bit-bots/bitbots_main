# Copyright 2022 Florian Vahl
# Copyright 2022 Kenji Brameld
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
from rclpy.node import Node

from soccer_vision_3d_msgs.msg import (
    BallArray, FieldBoundary, GoalpostArray, MarkingArray, ObstacleArray, RobotArray)
from soccer_vision_3d_rviz_markers.conversion import field_boundary_to_marker
from soccer_vision_3d_rviz_markers.conversion_array import (
    ball_array_to_marker_array, goalpost_array_to_marker_array,
    marking_array_to_marker_array, obstacle_array_to_marker_array, robot_array_to_marker_array)
from visualization_msgs.msg import Marker, MarkerArray


class SoccerVision3DMarkers(Node):

    def __init__(self):
        super().__init__('SoccerVision3DMarkers')

        # Declare parameters
        self.declare_parameter('ball_diameter', 0.10)
        self.declare_parameter('marking_segment_width', 0.05)
        self.declare_parameter('field_boundary_line_width', 0.02)

        # Create publishers
        self.balls_publisher = self.create_publisher(
            MarkerArray, 'visualization/balls', 10)
        self.field_boundary_publisher = self.create_publisher(
            Marker, 'visualization/field_boundary', 10)
        self.goalposts_publisher = self.create_publisher(
            MarkerArray, 'visualization/goalposts', 10)
        self.markings_publisher = self.create_publisher(
            MarkerArray, 'visualization/markings', 10)
        self.obstacles_publisher = self.create_publisher(
            MarkerArray, 'visualization/obstacles', 10)
        self.robots_publisher = self.create_publisher(
            MarkerArray, 'visualization/robots', 10)

        # Create subscriptions
        self.create_subscription(
            BallArray, 'soccer_vision_3d/balls', self.balls_cb, 10)
        self.create_subscription(
            FieldBoundary, 'soccer_vision_3d/field_boundary', self.field_boundary_cb, 10)
        self.create_subscription(
            GoalpostArray, 'soccer_vision_3d/goalposts', self.goalposts_cb, 10)
        self.create_subscription(
            MarkingArray, 'soccer_vision_3d/markings', self.markings_cb, 10)
        self.create_subscription(
            ObstacleArray, 'soccer_vision_3d/obstacles', self.obstacles_cb, 10)
        self.create_subscription(
            RobotArray, 'soccer_vision_3d/robots', self.robots_cb, 10)

    def balls_cb(self, msg: BallArray):
        self.balls_publisher.publish(ball_array_to_marker_array(
            msg, diameter=self.get_parameter('ball_diameter').value))

    def field_boundary_cb(self, msg: FieldBoundary):
        self.field_boundary_publisher.publish(field_boundary_to_marker(
            msg, width=self.get_parameter('field_boundary_line_width').value))

    def goalposts_cb(self, msg: GoalpostArray):
        self.goalposts_publisher.publish(goalpost_array_to_marker_array(msg))

    def markings_cb(self, msg: MarkingArray):
        self.markings_publisher.publish(marking_array_to_marker_array(
            msg, segment_width=self.get_parameter('marking_segment_width').value))

    def obstacles_cb(self, msg: ObstacleArray):
        self.obstacles_publisher.publish(obstacle_array_to_marker_array(msg))

    def robots_cb(self, msg: RobotArray):
        self.robots_publisher.publish(robot_array_to_marker_array(msg))


def main(args=None):
    rclpy.init(args=args)

    node = SoccerVision3DMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
