#!/usr/bin/env python3
"""Checks when the Pi Plus AMP kick starts and stops depending on the ball.

The node runs in this process with fake sensor data. The tests call the
callbacks directly, so no other ROS node is necessary.
"""

import threading
import time
from pathlib import Path
from unittest.mock import MagicMock

import pytest
import rclpy
import yaml  # type: ignore[import-untyped]
from bitbots_rl_motion.nodes.piplus_amp_kick_node import PiplusAmpKickNode
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import GoalResponse
from rclpy.duration import Duration
from sensor_msgs.msg import Imu, JointState

from bitbots_msgs.action import Kick
from bitbots_msgs.msg import RobotControlState

CONFIG = Path(__file__).parent.parent / "configs" / "piplus_amp_kick_model.yaml"


@pytest.fixture
def node():
    rclpy.init(args=["--ros-args", "--params-file", str(CONFIG)])
    kick_node = PiplusAmpKickNode()

    joints = yaml.safe_load(CONFIG.read_text())["piplus_amp_kick_node"]["ros__parameters"]["joints"]
    joint_state = JointState()
    joint_state.name = joints["ordered_relevant_joint_names"]
    joint_state.position = [float(p) for p in joints["walkready_state"]]
    joint_state.velocity = [0.0] * len(joint_state.name)
    kick_node._joint_handler._joint_state_callback(joint_state)
    imu = Imu()
    imu.orientation.w = 1.0
    kick_node._gyro_handler._imu_callback(imu)
    kick_node._gravity_handler._imu_callback(imu)
    kick_node._robot_state_handler._callback(RobotControlState(state=RobotControlState.CONTROLLABLE))
    # The tests do not have an odom frame. The kick direction only needs the robot yaw.
    kick_node._kick_handler._robot_yaw_odom = lambda timeout_s: 0.0  # type: ignore[method-assign]

    # Count the policy steps and the published joint goals
    session = kick_node._onnx_session
    kick_node._onnx_session = MagicMock(wraps=session)
    kick_node._joint_command_pub = MagicMock()

    yield kick_node

    kick_node.destroy_node()
    rclpy.shutdown()


def set_ball(node: PiplusAmpKickNode, age_s: float = 0.0, variance: float = 0.1) -> None:
    ball = PoseWithCovarianceStamped()
    # The ball is in the anchor frame, so no transform is necessary
    ball.header.frame_id = node.get_parameter("ball.anchor_frame").value
    stamp: TimeMsg = (node.get_clock().now() - Duration(seconds=age_s)).to_msg()
    ball.header.stamp = stamp
    ball.pose.pose.position.x = 0.5
    ball.pose.covariance[0] = variance
    ball.pose.covariance[7] = variance
    node._ball_handler._ball_callback(ball)


def run_control_steps(node: PiplusAmpKickNode, steps: int = 3) -> None:
    for _ in range(steps):
        node._timer_callback()


def start_kick(node: PiplusAmpKickNode, timeout: float = 10.0) -> tuple[threading.Thread, MagicMock, dict]:
    goal_handle = MagicMock()
    goal_handle.request = Kick.Goal(x=1.0, y=0.0, strength=4.0, timeout=timeout)
    goal_handle.is_cancel_requested = False
    output: dict = {}

    def execute():
        output["result"] = node._kick_handler._execute_kick(goal_handle)

    thread = threading.Thread(target=execute)
    thread.start()
    return thread, goal_handle, output


def test_goal_is_rejected_if_no_ball_was_received(node):
    logger = MagicMock()
    node.get_logger = lambda: logger

    assert node._kick_handler._goal_callback(Kick.Goal()) == GoalResponse.REJECT
    logger.warning.assert_called_once_with("Kick goal rejected because no ball estimate was received yet.")

    run_control_steps(node)
    assert not node._kick_handler.is_active()
    node._onnx_session.run.assert_not_called()
    node._joint_command_pub.publish.assert_not_called()


def test_kick_starts_with_a_valid_ball(node):
    set_ball(node)
    assert node._kick_handler._goal_callback(Kick.Goal()) == GoalResponse.ACCEPT
    thread, goal_handle, output = start_kick(node)
    deadline = time.monotonic() + 2.0
    while not node._kick_handler.is_active() and time.monotonic() < deadline:
        time.sleep(0.001)
    assert node._kick_handler.is_active()

    # The kick runs the policy and publishes joint goals
    run_control_steps(node, 3)
    assert node._onnx_session.run.call_count == 3
    assert node._joint_command_pub.publish.call_count == 3

    # End the goal
    goal_handle.is_cancel_requested = True
    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert output["result"].result == Kick.Result.ABORTED


def wait_until_active(node: PiplusAmpKickNode) -> None:
    deadline = time.monotonic() + 2.0
    while not node._kick_handler.is_active() and time.monotonic() < deadline:
        time.sleep(0.001)
    assert node._kick_handler.is_active()


def set_ball_distance(node: PiplusAmpKickNode, distance: float) -> None:
    # The tests have no transform into the distance frame, so the distance is given directly
    node._ball_handler.get_distance = lambda: distance  # type: ignore[method-assign]


def test_kick_without_time_limit_runs_while_the_ball_is_close(node):
    set_ball(node)
    set_ball_distance(node, node.get_parameter("ball.max_kick_distance").value - 0.1)
    # A goal without its own timeout uses command.kick_timeout, which means no time limit
    assert node.get_parameter("command.kick_timeout").value <= 0.0
    thread, goal_handle, output = start_kick(node, timeout=-1.0)
    wait_until_active(node)

    for _ in range(5):
        set_ball(node)
        run_control_steps(node, 1)
        time.sleep(0.05)
    assert node._kick_handler.is_active()
    assert thread.is_alive()

    goal_handle.is_cancel_requested = True
    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert output["result"].result == Kick.Result.ABORTED


def test_kick_stops_when_the_ball_is_too_far_away(node):
    set_ball(node)
    set_ball_distance(node, 0.5)
    thread, goal_handle, output = start_kick(node, timeout=-1.0)
    wait_until_active(node)
    run_control_steps(node, 2)
    assert node._kick_handler.is_active()

    set_ball_distance(node, node.get_parameter("ball.max_kick_distance").value + 0.1)
    run_control_steps(node, 1)
    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert not node._kick_handler.is_active()
    goal_handle.abort.assert_called_once()
    assert output["result"].result == Kick.Result.ABORTED


def test_kick_stops_when_the_ball_is_lost(node):
    set_ball(node)
    set_ball_distance(node, 0.5)
    thread, goal_handle, output = start_kick(node, timeout=-1.0)
    wait_until_active(node)

    # The ball estimate becomes too old
    set_ball(node, age_s=node.get_parameter("ball.timeout").value + 1.0)
    run_control_steps(node, 1)
    thread.join(timeout=2.0)
    assert not thread.is_alive()
    goal_handle.abort.assert_called_once()
    assert output["result"].result == Kick.Result.ABORTED
