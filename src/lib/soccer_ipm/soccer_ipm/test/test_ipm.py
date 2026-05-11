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

import math
from typing import Optional, Union

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from soccer_ipm.soccer_ipm import SoccerIPM
from soccer_ipm.utils import object_at_bottom_of_image
import soccer_vision_2d_msgs.msg as sv2dm
import soccer_vision_3d_msgs.msg as sv3dm
import soccer_vision_attribute_msgs.msg as sva
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Point2D

# Dummy CameraInfo Message
camera_info = CameraInfo(
        header=Header(
            frame_id='camera_optical_frame',
        ),
        width=2048,
        height=1536,
        binning_x=4,
        binning_y=4,
        k=[1338.64532, 0., 1024.0, 0., 1337.89746, 768.0, 0., 0., 1.])

img_center_x = camera_info.width / camera_info.binning_x // 2
img_center_y = camera_info.height / camera_info.binning_y // 2

# Custom types
SV2DARR = Union[
    sv2dm.BallArray,
    sv2dm.FieldBoundary,
    sv2dm.GoalpostArray,
    sv2dm.MarkingArray,
    sv2dm.ObstacleArray,
    sv2dm.RobotArray
]
SV3DARR = Union[
    sv3dm.BallArray,
    sv3dm.FieldBoundary,
    sv3dm.GoalpostArray,
    sv3dm.MarkingArray,
    sv3dm.ObstacleArray,
    sv3dm.RobotArray
]


def standard_ipm_test_case(
        input_msg_type: type,
        input_topic: str,
        input_msg: SV2DARR,
        output_msg_type: type,
        output_topic: str) -> tuple[SV3DARR, SV2DARR]:
    # Init ros
    rclpy.init()
    # Create IPM node
    node = SoccerIPM()
    # Create test node which comunicates with the IPM node
    test_node = Node('test_handler')
    # Create publishers to send data to the IPM node
    ball_pub = test_node.create_publisher(
        input_msg_type, input_topic, 10)
    camera_info_pub = test_node.create_publisher(
        CameraInfo, 'camera_info', 10)
    tf_pub = test_node.create_publisher(
        TFMessage, 'tf', 10)

    # Create a shared reference to the recived message in the local scope
    received_msg: list[Optional[output_msg_type]] = [None]

    # Create a callback with sets this reference
    def callback(msg):
        received_msg[0] = msg

    # Subscribe to IPM results
    test_node.create_subscription(
        output_msg_type, output_topic, callback, 10)

    # Create header message for the current time stamp in the camera frame
    header = Header(
        stamp=node.get_clock().now().to_msg(),
        frame_id='camera_optical_frame')

    # Create a dummy transform from the camera to the base_footprint frame
    tf = TransformStamped(
        header=header,
        child_frame_id='base_footprint',
    )
    tf.transform.translation.z = 1.0
    tf.transform.rotation.x = 0.0
    tf.transform.rotation.w = 1.0

    # Publish the dummy transform
    tf_pub.publish(TFMessage(
        transforms=[tf]
    ))
    # Spin the ipm to process the new data
    rclpy.spin_once(node, timeout_sec=0.1)

    # Send camera info message to the IPM
    camera_info.header.stamp = header.stamp
    camera_info_pub.publish(camera_info)
    # Spin the IPM to process the new data
    rclpy.spin_once(node, timeout_sec=0.1)

    # Send image space detection
    input_msg.header = header
    ball_pub.publish(input_msg)
    # Spin the IPM to process the new data
    rclpy.spin_once(node, timeout_sec=0.1)

    # Spin the test__node to recive the results from the IPM
    rclpy.spin_once(test_node, timeout_sec=0.1)

    # Assert that we recived a message
    assert received_msg[0] is not None

    # Clean shutdown of the nodes
    rclpy.shutdown()
    node.destroy_node()
    test_node.destroy_node()

    return received_msg[0], input_msg


def test_ipm_empty_ball():
    standard_ipm_test_case(
        sv2dm.BallArray,
        'balls_in_image',
        sv2dm.BallArray(),
        sv3dm.BallArray,
        'balls_relative')


def test_ipm_ball():
    # Create ball detection
    ball = sv2dm.Ball()
    ball.center.x = img_center_x
    ball.center.y = img_center_y
    ball.confidence = sva.Confidence(confidence=0.42)
    ball_detection = sv2dm.BallArray(balls=[ball])

    out, inp = standard_ipm_test_case(
        sv2dm.BallArray,
        'balls_in_image',
        ball_detection,
        sv3dm.BallArray,
        'balls_relative')

    # Assert that we recived the correct message
    assert len(out.balls) == 1, 'Wrong number of detections'
    assert out.header.stamp == inp.header.stamp, 'Time stamp got changed by the ipm'
    assert out.header.frame_id == 'base_footprint', \
        'Output frame is not "base_footprint"'
    ball_relative: sv3dm.Ball = out.balls[0]
    np.testing.assert_allclose(
        ball_relative.confidence.confidence,
        ball.confidence.confidence)
    np.testing.assert_allclose(
        [ball_relative.center.x, ball_relative.center.y, ball_relative.center.z],
        [0.0, 0.0, 0.0765])


def test_ipm_goalposts():
    # Create goalpost detection
    goalpost = sv2dm.Goalpost()
    goalpost.bb.size_x = 50.0
    goalpost.bb.size_y = 20.0
    # Footpoint in the cenetr of the image
    goalpost.bb.center.position.x = img_center_x
    goalpost.bb.center.position.y = img_center_y - goalpost.bb.size_y // 2
    goalpost.confidence = sva.Confidence(confidence=0.42)
    goalpost.attributes.side = sva.Goalpost.SIDE_LEFT
    goalpost.attributes.team = sva.Goalpost.TEAM_OPPONENT
    goalpost_detections = sv2dm.GoalpostArray(posts=[goalpost])

    out, inp = standard_ipm_test_case(
        sv2dm.GoalpostArray,
        'goal_posts_in_image',
        goalpost_detections,
        sv3dm.GoalpostArray,
        'goal_posts_relative')

    # Assert that we recived the correct message
    assert len(out.posts) == 1, 'Wrong number of detections'
    assert out.header.stamp == inp.header.stamp, 'Time stamp got changed by the ipm'
    assert out.header.frame_id == 'base_footprint', \
        'Output frame is not "base_footprint"'
    goalpost_relative: sv3dm.Goalpost = out.posts[0]
    np.testing.assert_allclose(
        goalpost_relative.confidence.confidence,
        goalpost.confidence.confidence)
    np.testing.assert_allclose(
        [
            goalpost_relative.bb.center.position.x,
            goalpost_relative.bb.center.position.y,
            goalpost_relative.bb.center.position.z
        ],
        [0.0, 0.0, 0.5])


def test_ipm_robots():
    # Create robot detection
    robot = sv2dm.Robot()
    robot.bb.size_x = 50.0
    robot.bb.size_y = 20.0
    # Footpoint in the cenetr of the image
    robot.bb.center.position.x = img_center_x
    robot.bb.center.position.y = img_center_y - robot.bb.size_y // 2
    robot.confidence = sva.Confidence(confidence=0.42)
    robot.attributes.state = sva.Robot.STATE_STANDING
    robot.attributes.team = sva.Robot.TEAM_OPPONENT
    robot.attributes.facing = sva.Robot.FACING_AWAY
    robot_detections = sv2dm.RobotArray(robots=[robot])

    out, inp = standard_ipm_test_case(
        sv2dm.RobotArray,
        'robots_in_image',
        robot_detections,
        sv3dm.RobotArray,
        'robots_relative')

    # Assert that we recived the correct message
    assert len(out.robots) == 1, 'Wrong number of detections'
    assert out.header.stamp == inp.header.stamp, 'Time stamp got changed by the ipm'
    assert out.header.frame_id == 'base_footprint', \
        'Output frame is not "base_footprint"'
    robot_relative: sv3dm.Robot = out.robots[0]
    assert robot_relative.attributes == robot.attributes, 'Attributes changed'
    np.testing.assert_allclose(
        robot_relative.confidence.confidence,
        robot.confidence.confidence)
    np.testing.assert_allclose(
        [
            robot_relative.bb.center.position.x,
            robot_relative.bb.center.position.y,
            robot_relative.bb.center.position.z
        ],
        [0.0, 0.0, 0.5])


def test_ipm_obstacles():
    # Create obstacle detection
    obstacle = sv2dm.Obstacle()
    obstacle.bb.size_x = 50.0
    obstacle.bb.size_y = 20.0
    # Footpoint in the cenetr of the image
    obstacle.bb.center.position.x = img_center_x
    obstacle.bb.center.position.y = img_center_y - obstacle.bb.size_y // 2
    obstacle.confidence = sva.Confidence(confidence=0.42)
    obstacle_detections = sv2dm.ObstacleArray(obstacles=[obstacle])

    out, inp = standard_ipm_test_case(
        sv2dm.ObstacleArray,
        'obstacles_in_image',
        obstacle_detections,
        sv3dm.ObstacleArray,
        'obstacles_relative')

    # Assert that we recived the correct message
    assert len(out.obstacles) == 1, 'Wrong number of detections'
    assert out.header.stamp == inp.header.stamp, 'Time stamp got changed by the ipm'
    assert out.header.frame_id == 'base_footprint', \
        'Output frame is not "base_footprint"'
    obstacle_relative: sv3dm.Obstacle = out.obstacles[0]
    np.testing.assert_allclose(
        obstacle_relative.confidence.confidence,
        obstacle.confidence.confidence)
    np.testing.assert_allclose(
        [
            obstacle_relative.bb.center.position.x,
            obstacle_relative.bb.center.position.y,
            obstacle_relative.bb.center.position.z
        ],
        [0.0, 0.0, 0.5])


def test_ipm_field_boundary():
    # Create field boundary detection
    field_boundary = sv2dm.FieldBoundary()
    field_boundary.confidence = sva.Confidence(confidence=0.42)
    field_boundary.points = [
        Point2D(
            x=float(img_center_x + i),
            y=float(img_center_y + i)) for i in range(100)]

    out, inp = standard_ipm_test_case(
        sv2dm.FieldBoundary,
        'field_boundary_in_image',
        field_boundary,
        sv3dm.FieldBoundary,
        'field_boundary_relative')

    # Assert that we recived the correct message
    assert len(out.points) == 100, 'Wrong number of detections'
    assert out.header.stamp == inp.header.stamp, 'Time stamp got changed by the ipm'
    assert out.header.frame_id == 'base_footprint', \
        'Output frame is not "base_footprint"'
    np.testing.assert_allclose(
        out.confidence.confidence,
        field_boundary.confidence.confidence)
    np.testing.assert_allclose(
        [out.points[0].x, out.points[0].y, out.points[0].z],
        [0.0, 0.0, 0.0])


def test_ipm_markings():
    # Create marking detections
    marking_array = sv2dm.MarkingArray()

    marking_intersection = sv2dm.MarkingIntersection()
    marking_intersection.confidence = sva.Confidence(confidence=0.42)
    marking_intersection.center = Point2D(
        x=float(img_center_x),
        y=float(img_center_y))
    marking_intersection.heading_rays = [
        0.0 * math.tau,
        0.5 * math.tau,
        0.25 * math.tau]
    marking_intersection.num_rays = 3
    marking_array.intersections.append(marking_intersection)

    marking_segment = sv2dm.MarkingSegment()
    marking_segment.confidence = sva.Confidence(confidence=0.42)
    marking_segment.start = Point2D(
        x=float(img_center_x),
        y=float(img_center_y))
    marking_segment.end = Point2D(
        x=float(img_center_x + 1.0),
        y=float(img_center_y))
    marking_array.segments.append(marking_segment)

    marking_ellipse = sv2dm.MarkingEllipse()
    marking_ellipse.confidence = sva.Confidence(confidence=0.42)
    marking_ellipse.bb.center.position = Point2D(
        x=float(img_center_x + 1.0),
        y=float(img_center_y))
    marking_ellipse.bb.size_x = 10.0
    marking_ellipse.bb.size_y = 10.0
    marking_ellipse.center = Point2D(
        x=float(img_center_x),
        y=float(img_center_y))
    marking_array.ellipses.append(marking_ellipse)

    out, inp = standard_ipm_test_case(
        sv2dm.MarkingArray,
        'markings_in_image',
        marking_array,
        sv3dm.MarkingArray,
        'markings_relative')

    # Assert that we recived the correct message
    assert len(out.intersections) == 1, 'Wrong number of detections'
    assert len(out.ellipses) == 1, 'Wrong number of detections'
    assert len(out.segments) == 1, 'Wrong number of detections'
    assert out.header.stamp == inp.header.stamp, 'Time stamp got changed by the ipm'
    assert out.header.frame_id == 'base_footprint', \
        'Output frame is not "base_footprint"'

    mapped_ellipse: sv3dm.MarkingEllipse = out.ellipses[0]
    mapped_intersection: sv3dm.MarkingIntersection = out.intersections[0]
    mapped_segment: sv3dm.MarkingSegment = out.segments[0]

    assert math.isclose(mapped_ellipse.center.position.x, 0), 'Ellipse X position off'
    assert math.isclose(mapped_ellipse.center.position.y, 0), 'Ellipse Y position off'
    assert math.isclose(
        mapped_ellipse.confidence.confidence,
        inp.ellipses[0].confidence.confidence, abs_tol=1e-5), 'Confidence is off'
    assert math.isclose(mapped_ellipse.diameter, 0.035857145, abs_tol=1e-5), 'Diameter is wrong'

    assert math.isclose(mapped_segment.start.x, 0), 'Segment X position off'
    assert math.isclose(mapped_segment.start.y, 0), 'Segment Y position off'
    assert math.isclose(
        mapped_segment.confidence.confidence,
        inp.segments[0].confidence.confidence, abs_tol=1e-5), 'Confidence is off'

    assert math.isclose(mapped_intersection.center.x, 0), 'Intersection X position off'
    assert math.isclose(mapped_intersection.center.y, 0), 'Intersection Y position off'
    assert mapped_intersection.num_rays == inp.intersections[0].num_rays, \
        'Wrong number of rays in intersection'

    heading_vectors = np.array([(
            heading_vector.x,
            heading_vector.y,
            heading_vector.z
        ) for heading_vector in mapped_intersection.rays])

    # Normalize vectors to length 1
    heading_vectors /= np.linalg.norm(heading_vectors, axis=1)

    # Check relationships between the rays
    assert math.isclose(np.dot(*heading_vectors[[0, 1]]), -1), \
        'Ray one and two are not in opposite directions'
    assert math.isclose(np.dot(*heading_vectors[[0, 2]]), 0), \
        'Ray one and three are not orthogonal'

    common_normal = np.cross(*heading_vectors[[0, 2]])
    assert math.isclose(np.dot(common_normal, heading_vectors[1]), 0), \
        'Not all rays are on the same plane'


def test_out_of_image():
    bottom = camera_info.height / camera_info.binning_y
    assert not object_at_bottom_of_image(camera_info, -1.0, img_center_y)
    assert not object_at_bottom_of_image(camera_info, 0.0, 1.0)
    assert not object_at_bottom_of_image(camera_info, bottom - 1, 1.0)
    assert not object_at_bottom_of_image(camera_info, bottom, 1.0)
    assert object_at_bottom_of_image(camera_info, bottom + 1, 1.0)
    assert object_at_bottom_of_image(camera_info, bottom, 0.5)
