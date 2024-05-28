from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import tf2_ros as tf2
from bitbots_path_planning.controller import Controller
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import PoseStamped


def test_default_setup(snapshot, node, tf2_buffer):
    controller = setup_controller(node, tf2_buffer)
    parameter_keys = [
        "config_base_footprint_frame",
        "config_carrot_distance",
        "config_max_rotation_vel",
        "config_max_vel_x",
        "config_max_vel_y",
        "config_min_vel_x",
        "config_orient_to_goal_distance",
        "config_rotation_slow_down_factor",
        "config_rotation_i_factor",
        "config_smoothing_tau",
        "config_translation_slow_down_factor",
    ]
    parameters = {p: getattr(controller, p) for p in parameter_keys}

    assert controller.angular_error_accumulator == 0
    assert controller.last_cmd_vel == Twist()
    assert parameters == snapshot


def test_step_limits_forward_x_velocity(node, tf2_buffer, pose_opponent_goal):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_opponent_goal))

    assert controller.last_cmd_vel.linear.x == controller.config_max_vel_x
    assert controller.last_cmd_vel.linear.y == pytest.approx(0)
    assert controller.last_cmd_vel.linear.z == 0


def test_step_limits_backward_x_velocity(node, tf2_buffer, pose_own_goal):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_own_goal))

    assert controller.last_cmd_vel.linear.x == controller.config_min_vel_x
    assert controller.last_cmd_vel.linear.y == pytest.approx(0)
    assert controller.last_cmd_vel.linear.z == 0


def test_step_limits_forward_y_velocity(node, tf2_buffer, pose_left_line):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_left_line))

    assert controller.last_cmd_vel.linear.x == pytest.approx(0)
    assert controller.last_cmd_vel.linear.y == controller.config_max_vel_y
    assert controller.last_cmd_vel.linear.z == 0


def test_step_limits_backward_y_velocity(node, tf2_buffer, pose_right_line):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_right_line))

    assert controller.last_cmd_vel.linear.x == pytest.approx(0)
    assert controller.last_cmd_vel.linear.y == -controller.config_max_vel_y
    assert controller.last_cmd_vel.linear.z == 0


def test_step_limits_forward_xy_velocities(node, tf2_buffer, pose_opponent_corner):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_opponent_corner))

    assert controller.last_cmd_vel.linear.x == pytest.approx(0.12)
    assert controller.last_cmd_vel.linear.y == controller.config_max_vel_y
    assert controller.last_cmd_vel.linear.z == 0


def test_step_limits_backward_xy_velocities(node, tf2_buffer, pose_own_corner):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_own_corner))

    assert controller.last_cmd_vel.linear.x == pytest.approx(-0.12)
    assert controller.last_cmd_vel.linear.y == -controller.config_max_vel_y
    assert controller.last_cmd_vel.linear.z == 0


def test_step_limits_rotation(node, tf2_buffer, pose_left_line, pose_right_line):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_left_line))
    assert controller.last_cmd_vel.angular.z == controller.config_max_rotation_vel

    controller.last_update_time = None
    controller.step(path_to(pose_right_line))
    assert controller.last_cmd_vel.angular.z == -controller.config_max_rotation_vel


def test_step_cmd_vel_smoothing(snapshot, node, tf2_buffer, pose_opponent_corner):
    controller = setup_controller(node, tf2_buffer)
    controller.config_smoothing_tau = 0.5
    controller.last_update_time = Time(seconds=0)

    controller.last_cmd_vel.linear.x = controller.config_max_vel_x
    controller.last_cmd_vel.linear.y = controller.config_max_vel_y
    controller.last_cmd_vel.angular.z = controller.config_max_rotation_vel

    controller.step(path_to(pose_opponent_corner))

    assert str(controller.last_cmd_vel) == snapshot


def setup_controller(node: Node, buffer: tf2.Buffer) -> Controller:
    return Controller(node, buffer)


def path_to(pose: PoseStamped) -> Path:
    path = Path()
    path.header.frame_id = "map"
    path.poses = [pose]

    return path


@pytest.fixture
def tf2_buffer(pose_center_point: PoseStamped) -> tf2.Buffer:
    buffer = Mock(tf2.Buffer)
    buffer.transform.return_value = pose_center_point

    return buffer


@pytest.fixture
def pose_center_point() -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0

    return pose


@pytest.fixture
def pose_own_goal() -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = -4.5
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0

    return pose


@pytest.fixture
def pose_opponent_goal() -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = 4.5
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0

    return pose


@pytest.fixture
def pose_own_corner() -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = -4.5
    pose.pose.position.y = -3.0
    pose.pose.position.z = 0.0

    return pose


@pytest.fixture
def pose_opponent_corner() -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = 4.5
    pose.pose.position.y = 3.0
    pose.pose.position.z = 0.0

    return pose


@pytest.fixture
def pose_right_line() -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = 0.0
    pose.pose.position.y = -3.0
    pose.pose.position.z = 0.0

    return pose


@pytest.fixture
def pose_left_line() -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = 0.0
    pose.pose.position.y = 3.0
    pose.pose.position.z = 0.0

    return pose


@pytest.fixture
def node() -> Node:
    node = Mock(Node)
    node.get_parameter = lambda key: SimpleNamespace(value=key)
    node.declare_parameter = lambda _, default_value: SimpleNamespace(value=default_value)

    node.get_logger.return_value.debug = print
    node.get_clock.return_value.now.return_value = Time(seconds=1)

    return node
