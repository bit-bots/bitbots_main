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

from geometry_msgs.msg import Point, Vector3
from soccer_vision_3d_msgs.msg import (
    Ball, FieldBoundary, Goalpost, MarkingEllipse, MarkingIntersection, MarkingSegment, Obstacle,
    Robot)
from soccer_vision_3d_rviz_markers.conversion import (
    ball_to_marker, conf_to_alpha, field_boundary_to_marker, goalpost_to_marker,
    marking_ellipse_to_marker, marking_intersection_to_marker, marking_segment_to_marker,
    obstacle_to_marker, robot_to_markers)
from soccer_vision_attribute_msgs.msg import Confidence
from visualization_msgs.msg import Marker


def test_conf_to_alpha():

    confidence = Confidence(confidence=Confidence.CONFIDENCE_UNKNOWN)
    assert conf_to_alpha(confidence) == 1.0

    confidence = Confidence(confidence=0.7)
    assert conf_to_alpha(confidence) == 0.7


def test_ball_to_marker():

    ball = Ball()
    ball.center.x = 1.0
    ball.center.y = 2.0
    ball.center.z = 3.0
    ball.confidence.confidence = 0.5
    marker = ball_to_marker(ball, diameter=0.10)

    assert marker.type == Marker.MESH_RESOURCE
    assert marker.action == Marker.MODIFY
    assert marker.pose.position.x == 1.0
    assert marker.pose.position.y == 2.0
    assert marker.pose.position.z == 3.0
    # Scale (x, y, z) should be equal to the diameter parameter
    assert marker.scale.x == 0.10
    assert marker.scale.y == 0.10
    assert marker.scale.z == 0.10
    assert marker.color.r == 0.0
    assert marker.color.g == 0.0
    assert marker.color.b == 0.0
    assert marker.color.a == 0.5
    assert marker.mesh_resource == 'package://soccer_vision_3d_rviz_markers/blender/ball.dae'
    assert marker.mesh_use_embedded_materials is True


def test_field_boundary_to_marker():
    field_boundary = FieldBoundary()
    field_boundary.points = [
        Point(x=0.1, y=0.2, z=0.3), Point(x=0.4, y=0.5, z=0.6)]
    field_boundary.confidence.confidence = 0.6
    marker = field_boundary_to_marker(field_boundary, width=0.15)

    assert marker.type == Marker.LINE_STRIP
    assert marker.action == Marker.MODIFY
    assert marker.points[0].x == 0.1
    assert marker.points[0].y == 0.2
    assert marker.points[0].z == 0.3
    assert marker.points[1].x == 0.4
    assert marker.points[1].y == 0.5
    assert marker.points[1].z == 0.6
    assert marker.scale.x == 0.15
    assert marker.color.r == 0.0
    assert marker.color.g == 1.0
    assert marker.color.b == 0.0
    assert marker.color.a == 0.6


def test_goalpost_to_marker():
    goalpost = Goalpost()
    goalpost.bb.center.position.x = 0.1
    goalpost.bb.center.position.y = 0.2
    goalpost.bb.center.position.z = 0.3
    goalpost.bb.center.orientation.x = 0.128
    goalpost.bb.center.orientation.y = 0.145
    goalpost.bb.center.orientation.z = 0.269
    goalpost.bb.center.orientation.w = 0.944
    goalpost.bb.size.x = 0.4
    goalpost.bb.size.y = 0.5
    goalpost.bb.size.z = 0.6
    goalpost.confidence.confidence = 0.7
    marker = goalpost_to_marker(goalpost)

    assert marker.type == Marker.CYLINDER
    assert marker.pose.position.x == 0.1
    assert marker.pose.position.y == 0.2
    assert marker.pose.position.z == 0.3
    assert marker.pose.orientation.x == 0.128
    assert marker.pose.orientation.y == 0.145
    assert marker.pose.orientation.z == 0.269
    assert marker.pose.orientation.w == 0.944
    assert marker.scale.x == 0.4
    assert marker.scale.y == 0.5
    assert marker.scale.z == 0.6
    assert marker.color.r == 1.0
    assert marker.color.g == 1.0
    assert marker.color.b == 1.0
    assert marker.color.a == 0.7


def test_marking_ellipse_to_marker():
    marking_ellipse = MarkingEllipse()
    marking_ellipse.diameter = 0.5
    marking_ellipse.center.position.x = 0.1
    marking_ellipse.center.position.y = 0.2
    marking_ellipse.center.position.z = 0.3
    marking_ellipse.center.orientation.x = 0.128
    marking_ellipse.center.orientation.y = 0.145
    marking_ellipse.center.orientation.z = 0.269
    marking_ellipse.center.orientation.w = 0.944
    marking_ellipse.confidence.confidence = 0.7
    marker = marking_ellipse_to_marker(marking_ellipse)

    assert marker.type == Marker.CYLINDER
    assert marker.pose.position.x == 0.1
    assert marker.pose.position.y == 0.2
    assert marker.pose.position.z == 0.3
    assert marker.pose.orientation.x == 0.128
    assert marker.pose.orientation.y == 0.145
    assert marker.pose.orientation.z == 0.269
    assert marker.pose.orientation.w == 0.944
    assert marker.scale.x == 0.5  # Should be same as diameter
    assert marker.scale.y == 0.5  # Should be same as diameter
    # Set z scale to be very small, but can't be zero because a cylinder needs a height to be
    # valid. Also, setting this to a lower value makes the cylinder too thin and the marker
    # starts becoming transparent. 0.005 is a compromise.
    assert marker.scale.z == 0.005
    assert marker.color.r == 1.0
    assert marker.color.g == 1.0
    assert marker.color.b == 1.0
    assert marker.color.a == 0.7


def test_marking_intersection_to_marker():
    marking_intersection = MarkingIntersection()
    marking_intersection.center.x = 0.1
    marking_intersection.center.y = 0.2
    marking_intersection.center.z = 0.3
    marking_intersection.num_rays = 2
    marking_intersection.rays = [Vector3(x=1.0), Vector3(y=1.0)]
    marking_intersection.confidence.confidence = 0.7
    marker = marking_intersection_to_marker(marking_intersection)

    assert marker.type == Marker.LINE_LIST
    assert marker.pose.position.x == 0.1
    assert marker.pose.position.y == 0.2
    assert marker.pose.position.z == 0.3
    # 0.1m length vector in direction of 1st ray
    assert Point(x=0.1) in marker.points
    # 0.1m length vector in direction of 2nd ray
    assert Point(y=0.1) in marker.points
    assert marker.points.count(Point()) == 2  # center
    assert marker.scale.x == 0.02  # 0.02m line width
    assert marker.color.r == 1.0
    assert marker.color.g == 0.0
    assert marker.color.b == 1.0
    assert marker.color.a == 0.7


def test_marking_segment_to_marker():
    marking_segment = MarkingSegment()
    marking_segment.start.x = 0.1
    marking_segment.start.y = 0.2
    marking_segment.start.z = 0.3
    marking_segment.end.x = 0.4
    marking_segment.end.y = 0.5
    marking_segment.end.z = 0.6
    marking_segment.confidence.confidence = 0.7
    marker = marking_segment_to_marker(marking_segment, width=0.05)

    assert marker.type == Marker.LINE_STRIP
    assert marker.points[0].x == 0.1
    assert marker.points[0].y == 0.2
    assert marker.points[0].z == 0.3
    assert marker.points[1].x == 0.4
    assert marker.points[1].y == 0.5
    assert marker.points[1].z == 0.6
    assert marker.scale.x == 0.05
    assert marker.color.r == 1.0
    assert marker.color.g == 1.0
    assert marker.color.b == 1.0
    assert marker.color.a == 0.7


def test_obstacle_to_marker():
    obstacle = Obstacle()
    obstacle.bb.center.position.x = 0.1
    obstacle.bb.center.position.y = 0.2
    obstacle.bb.center.position.z = 0.3
    obstacle.bb.center.orientation.x = 0.128
    obstacle.bb.center.orientation.y = 0.145
    obstacle.bb.center.orientation.z = 0.269
    obstacle.bb.center.orientation.w = 0.944
    obstacle.bb.size.x = 0.4
    obstacle.bb.size.y = 0.5
    obstacle.bb.size.z = 0.6
    obstacle.confidence.confidence = 0.7

    marker = obstacle_to_marker(obstacle)
    assert marker.type == Marker.CUBE
    assert marker.pose.position.x == 0.1
    assert marker.pose.position.y == 0.2
    assert marker.pose.position.z == 0.3
    assert marker.pose.orientation.x == 0.128
    assert marker.pose.orientation.y == 0.145
    assert marker.pose.orientation.z == 0.269
    assert marker.pose.orientation.w == 0.944
    assert marker.scale.x == 0.4
    assert marker.scale.y == 0.5
    assert marker.scale.z == 0.6
    assert marker.color.r == 0.0
    assert marker.color.g == 0.0
    assert marker.color.b == 0.0
    assert marker.color.a == 0.7


def test_robot_to_markers():
    robot = Robot()
    robot.bb.center.position.x = 0.1
    robot.bb.center.position.y = 0.2
    robot.bb.center.position.z = 0.3
    robot.bb.center.orientation.x = 0.128
    robot.bb.center.orientation.y = 0.145
    robot.bb.center.orientation.z = 0.269
    robot.bb.center.orientation.w = 0.944
    robot.bb.size.x = 0.4
    robot.bb.size.y = 0.5
    robot.bb.size.z = 0.6
    robot.confidence.confidence = 0.7
    robot.attributes.player_number = 1
    robot.attributes.team = robot.attributes.TEAM_UNKNOWN
    robot.attributes.state = robot.attributes.STATE_KICKING
    robot.attributes.facing = robot.attributes.FACING_LEFT
    markers = robot_to_markers(robot)

    assert len(markers) == 2

    assert markers[0].type == Marker.CUBE
    assert markers[0].pose.position.x == 0.1
    assert markers[0].pose.position.y == 0.2
    assert markers[0].pose.position.z == 0.3
    assert markers[0].pose.orientation.x == 0.128
    assert markers[0].pose.orientation.y == 0.145
    assert markers[0].pose.orientation.z == 0.269
    assert markers[0].pose.orientation.w == 0.944
    assert markers[0].scale.x == 0.4
    assert markers[0].scale.y == 0.5
    assert markers[0].scale.z == 0.6
    assert markers[0].color.r == 1.0
    assert markers[0].color.g == 1.0
    assert markers[0].color.b == 1.0
    assert markers[0].color.a == 0.7

    assert markers[1].type == Marker.TEXT_VIEW_FACING
    assert markers[1].pose.position.x == 0.1
    assert markers[1].pose.position.y == 0.2
    assert markers[1].pose.position.z == 0.3 + 0.3 + 0.1  # center.z + scale.z/2 + 0.1m text offset
    assert markers[1].pose.orientation.x == 0.128
    assert markers[1].pose.orientation.y == 0.145
    assert markers[1].pose.orientation.z == 0.269
    assert markers[1].pose.orientation.w == 0.944
    assert markers[1].scale.z == 0.2  # Font size
    assert markers[1].color.r == 1.0
    assert markers[1].color.g == 1.0
    assert markers[1].color.b == 1.0
    assert markers[1].color.a == 0.7
    assert markers[1].text == '1'  # Player number


def test_own_team_robot_to_markers():
    robot = Robot()
    robot.attributes.team = robot.attributes.TEAM_OWN
    robot.attributes.player_number = 1
    markers = robot_to_markers(robot)

    assert len(markers) == 2

    assert markers[0].color.r == 0.0
    assert markers[0].color.g == 1.0
    assert markers[0].color.b == 0.0
    assert markers[1].color.r == 0.0
    assert markers[1].color.g == 1.0
    assert markers[1].color.b == 0.0


def test_opponent_team_robot_to_markers():
    robot = Robot()
    robot.attributes.team = robot.attributes.TEAM_OPPONENT
    robot.attributes.player_number = 1
    markers = robot_to_markers(robot)

    assert len(markers) == 2

    assert markers[0].color.r == 1.0
    assert markers[0].color.g == 0.0
    assert markers[0].color.b == 0.0
    assert markers[1].color.r == 1.0
    assert markers[1].color.g == 0.0
    assert markers[1].color.b == 0.0
