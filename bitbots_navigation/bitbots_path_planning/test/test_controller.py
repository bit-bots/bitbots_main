import math
from unittest.mock import Mock

import pytest
import tf2_ros as tf2
from bitbots_path_planning.controller import Controller
from bitbots_path_planning.path_planning_parameters import bitbots_path_planning as gen_params
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import PoseStamped


def test_default_setup(snapshot, node, tf2_buffer, config):
    controller = setup_controller(node, tf2_buffer)
    parameter_keys = [
        "carrot_distance",
        "max_rotation_vel",
        "max_vel_x",
        "max_vel_y",
        "min_vel_x",
        "orient_to_goal_distance",
        "rotation_slow_down_factor",
        "rotation_i_factor",
        "smoothing_tau",
        "translation_slow_down_factor",
    ]
    parameters = {p: getattr(config.controller, p) for p in parameter_keys}

    assert controller.angular_error_accumulator == 0
    assert controller.last_cmd_vel == Twist()
    assert parameters == snapshot


def test_step_limits_forward_x_velocity(node, tf2_buffer, config, pose_opponent_goal):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_opponent_goal))

    assert controller.last_cmd_vel.linear.x == pytest.approx(config.controller.max_vel_x)
    assert controller.last_cmd_vel.linear.y == pytest.approx(0)
    assert controller.last_cmd_vel.linear.z == 0


def test_step_limits_backward_x_velocity(node, tf2_buffer, config, pose_own_goal):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_own_goal))

    assert controller.last_cmd_vel.linear.x == pytest.approx(config.controller.min_vel_x)
    assert controller.last_cmd_vel.linear.y == pytest.approx(0)
    assert controller.last_cmd_vel.linear.z == 0


def test_step_limits_forward_y_velocity(node, tf2_buffer, config, pose_left_line):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_left_line))

    assert controller.last_cmd_vel.linear.x == pytest.approx(0)
    assert controller.last_cmd_vel.linear.y == pytest.approx(config.controller.max_vel_y)
    assert controller.last_cmd_vel.linear.z == 0


def test_step_limits_backward_y_velocity(node, tf2_buffer, config, pose_right_line):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_right_line))

    assert controller.last_cmd_vel.linear.x == pytest.approx(0)
    assert controller.last_cmd_vel.linear.y == pytest.approx(-config.controller.max_vel_y)
    assert controller.last_cmd_vel.linear.z == 0


def test_step_limits_forward_xy_velocities(node, tf2_buffer, config, pose_opponent_corner):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_opponent_corner))

    goal_heading_angle = math.atan2(pose_opponent_corner.pose.position.y, pose_opponent_corner.pose.position.x)
    walk_command_angle = math.atan2(controller.last_cmd_vel.linear.y, controller.last_cmd_vel.linear.x)

    assert goal_heading_angle == pytest.approx(walk_command_angle)

    walk_command_speed = math.hypot(controller.last_cmd_vel.linear.x, controller.last_cmd_vel.linear.y)

    assert walk_command_speed < max(config.controller.max_vel_x, config.controller.max_vel_y)
    assert controller.last_cmd_vel.linear.z == 0


def test_step_limits_backward_xy_velocities(node, tf2_buffer, config, pose_own_corner):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_own_corner))

    goal_heading_angle = math.atan2(pose_own_corner.pose.position.y, pose_own_corner.pose.position.x)
    walk_command_angle = math.atan2(controller.last_cmd_vel.linear.y, controller.last_cmd_vel.linear.x)

    assert goal_heading_angle == pytest.approx(walk_command_angle)

    walk_command_speed = math.hypot(controller.last_cmd_vel.linear.x, controller.last_cmd_vel.linear.y)

    assert walk_command_speed < abs(config.controller.min_vel_x)
    assert walk_command_speed < config.controller.max_vel_y
    assert controller.last_cmd_vel.linear.z == 0


def test_step_limits_rotation(node, tf2_buffer, config, pose_left_line, pose_right_line):
    controller = setup_controller(node, tf2_buffer)

    controller.step(path_to(pose_left_line))
    assert controller.last_cmd_vel.angular.z == pytest.approx(config.controller.max_rotation_vel)

    controller.last_update_time = None
    controller.step(path_to(pose_right_line))
    assert controller.last_cmd_vel.angular.z == pytest.approx(-config.controller.max_rotation_vel)


def test_step_cmd_vel_smoothing(snapshot, node, tf2_buffer, config, pose_opponent_corner):
    controller = setup_controller(node, tf2_buffer)
    controller.node.config.controller.smoothing_tau = 0.5
    controller.last_update_time = Time(seconds=0)

    controller.last_cmd_vel.linear.x = config.controller.max_vel_x
    controller.last_cmd_vel.linear.y = config.controller.max_vel_y
    controller.last_cmd_vel.angular.z = config.controller.max_rotation_vel

    controller.step(path_to(pose_opponent_corner))

    assert str(controller.last_cmd_vel) == snapshot


def setup_controller(node: Node, buffer: tf2.BufferInterface) -> Controller:
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
def config() -> gen_params.Params:
    return gen_params.Params()


@pytest.fixture
def node(config) -> Node:
    node = Mock(Node)

    node.config = config

    node.get_logger.return_value.debug = print
    node.get_clock.return_value.now.return_value = Time(seconds=1)

    return node
