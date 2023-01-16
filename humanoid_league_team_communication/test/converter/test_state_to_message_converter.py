from unittest.mock import Mock

import numpy
import pytest
from std_msgs.msg import Header
from geometry_msgs.msg import (Point, PointStamped, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped,
                               Quaternion, Twist, Vector3)
from humanoid_league_team_communication.converter.robocup_protocol_converter import RobocupProtocolConverter
from humanoid_league_team_communication.humanoid_league_team_communication import HumanoidLeagueTeamCommunication
from humanoid_league_team_communication.robocup_extension_pb2 import *
from rclpy.time import Time

from humanoid_league_msgs.msg import GameState, ObstacleRelative, ObstacleRelativeArray, PoseWithCertainty, Strategy

own_team_id = 1
validity_checker_valid = Mock(return_value=True)
validity_checker_expired = Mock(return_value=False)

def test_convert_empty_state(snapshot, state):
    result = convert_to_message(state)

    assert result.current_pose.player_id == state.player_id
    assert result.current_pose.team == state.team_id
    assert str(result) == snapshot


def test_convert_gamestate(state_with_gamestate):
    gamestate = state_with_gamestate.gamestate

    gamestate.penalized = False
    result = convert_to_message(state_with_gamestate)
    assert result.state == State.UNPENALISED

    gamestate.penalized = True
    result = convert_to_message(state_with_gamestate)
    assert result.state == State.PENALISED

    assert validity_checker_valid.call_count == 2
    validity_checker_valid.assert_called_with(gamestate.header.stamp)


def test_convert_gamestate_expired_headers(state_with_gamestate):
    message = Message()
    message.state = State.UNPENALISED

    result = convert_to_message(state_with_gamestate, message, is_state_expired=True)

    assert result.state == State.UNKNOWN_STATE


def test_convert_current_pose(snapshot, state_with_current_pose):
    result = convert_to_message(state_with_current_pose)

    assert str(result.current_pose.position) == snapshot
    assert str(result.current_pose.covariance) == snapshot

    validity_checker_valid.assert_called_with(state_with_current_pose.pose.header.stamp)


def test_convert_current_pose_expired_headers(state_with_current_pose):
    message = Message()
    message.current_pose.position.x = 3.0

    result = convert_to_message(state_with_current_pose, message, is_state_expired=True)

    assert result.current_pose.position.x == 3.0
    assert result.current_pose.covariance.x.x == 100
    assert result.current_pose.covariance.y.y == 100
    assert result.current_pose.covariance.z.z == 100


def test_convert_walk_command(state_with_walk_command):
    result = convert_to_message(state_with_walk_command)

    assert result.walk_command.x == state_with_walk_command.cmd_vel.linear.x
    assert result.walk_command.y == state_with_walk_command.cmd_vel.linear.y
    assert result.walk_command.z == state_with_walk_command.cmd_vel.angular.z

    validity_checker_valid.assert_called_with(state_with_walk_command.cmd_vel_time)


def test_convert_walk_command_expired_headers(state_with_walk_command):
    message = Message()
    message.walk_command.x = 3.0

    result = convert_to_message(state_with_walk_command, message, is_state_expired=True)

    assert result.walk_command.x == 3.0
    assert result.walk_command.y == 0
    assert result.walk_command.z == 0


def test_convert_target_pose(snapshot, state_with_target_pose):
    result = convert_to_message(state_with_target_pose)

    assert str(result.target_pose) == snapshot

    validity_checker_valid.assert_called_with(state_with_target_pose.move_base_goal.header.stamp)


def test_convert_target_pose_expired_headers(state_with_target_pose):
    message = Message()
    message.target_pose.position.x = 3.0

    result = convert_to_message(state_with_target_pose, message, is_state_expired=True)

    assert result.target_pose.position.x == 3.0
    assert result.target_pose.position.y == 0
    assert result.target_pose.position.z == 0


def test_convert_ball_position(snapshot, state_with_ball_pose):
    result = convert_to_message(state_with_ball_pose)

    assert str(result.ball) == snapshot

    validity_checker_valid.assert_called_with(state_with_ball_pose.ball.header.stamp)


def test_convert_ball_position_expired_headers(state_with_ball_pose):
    result = convert_to_message(state_with_ball_pose, is_state_expired=True)

    assert result.ball.covariance.x.x == 100
    assert result.ball.covariance.y.y == 100
    assert result.ball.covariance.z.z == 100


def test_convert_obstacles_to_robots(snapshot, state_with_obstacles):
    result = convert_to_message(state_with_obstacles)

    assert str(result.others) == snapshot
    assert result.other_robot_confidence == snapshot

    validity_checker_valid.assert_called_with(state_with_obstacles.obstacles.header.stamp)


def test_convert_obstacles_to_robots_expired_headers(state_with_obstacles):
    robot = Robot()
    robot.player_id = 2
    message = Message()
    message.others.append(robot)

    result = convert_to_message(state_with_obstacles, message, is_state_expired=True)

    assert result.others[0].player_id == robot.player_id
    assert len(result.others) == 1


def test_convert_strategy(state_with_strategy):
    result = convert_to_message(state_with_strategy)

    assert result.role == Role.ROLE_STRIKER
    assert result.action == Action.ACTION_KICKING
    assert result.offensive_side == OffensiveSide.SIDE_RIGHT

    validity_checker_valid.assert_called_with(state_with_strategy.strategy_time)


def test_convert_strategy_expired_headers(state_with_strategy):
    message = Message()
    message.role = Role.ROLE_DEFENDER

    result = convert_to_message(state_with_strategy, message, is_state_expired=True)

    assert result.role == Role.ROLE_DEFENDER
    assert result.action == Action.ACTION_UNDEFINED
    assert result.offensive_side == OffensiveSide.SIDE_UNDEFINED


def test_convert_time_to_ball(state_with_time_to_ball):
    result = convert_to_message(state_with_time_to_ball)

    assert pytest.approx(result.time_to_ball) == state_with_time_to_ball.time_to_ball

    validity_checker_valid.assert_called_with(state_with_time_to_ball.time_to_ball_time)


def test_convert_time_to_ball_calculated(snapshot, state_with_ball_and_robot_pose):
    result = convert_to_message(state_with_ball_and_robot_pose)

    assert result.time_to_ball == snapshot

    validity_checker_valid.assert_called_with(state_with_ball_and_robot_pose.pose.header.stamp)
    validity_checker_valid.assert_called_with(state_with_ball_and_robot_pose.ball.header.stamp)


def test_convert_time_to_ball_expired_headers(state_with_time_to_ball):
    message = Message()
    message.time_to_ball = 16.5

    result = convert_to_message(state_with_time_to_ball, message, is_state_expired=True)

    assert pytest.approx(result.time_to_ball) == 9999.0


def convert_to_message(team_data_state, message: Message = None, is_state_expired=False):
    message = message if message else Message()
    validity_checker = validity_checker_expired if is_state_expired else validity_checker_valid
    return RobocupProtocolConverter().convert_to_message(team_data_state, message, validity_checker)


@pytest.fixture
def state_with_ball_and_robot_pose(state_with_current_pose, state_with_ball_pose):
    state_with_current_pose.ball = state_with_ball_pose.ball
    return state_with_current_pose


@pytest.fixture
def state_with_time_to_ball(state):
    state.time_to_ball = 6.4
    state.time_to_ball_time = Mock(Time)

    return state


@pytest.fixture
def state_with_strategy(state):
    state.strategy = Strategy()
    state.strategy.role = Strategy.ROLE_STRIKER
    state.strategy.action = Strategy.ACTION_KICKING
    state.strategy.offensive_side = Strategy.SIDE_RIGHT
    state.strategy_time = Mock(Time)

    return state


@pytest.fixture
def state_with_obstacles(state):
    state.obstacles = Mock(ObstacleRelativeArray)
    state.obstacles.header = Header()
    state.obstacles.obstacles = [
        relative_obstacle(ObstacleRelative.ROBOT_CYAN, 2),
        relative_obstacle(ObstacleRelative.ROBOT_MAGENTA, 3),
        relative_obstacle(ObstacleRelative.ROBOT_UNDEFINED, 4),
        relative_obstacle(ObstacleRelative.HUMAN),
        relative_obstacle(ObstacleRelative.POLE),
    ]

    return state


@pytest.fixture
def state_with_ball_pose(state):
    point = Point(x=4.6, y=2.8, z=2.9)
    state.ball = PointStamped(point=point)
    state.ball.header = Header()
    state.ball_velocity = (1.1, 0.2, 0.3)
    state.ball_covariance = covariance_list()

    return state


@pytest.fixture
def state_with_walk_command(state):
    state.cmd_vel = Mock(Twist)
    state.cmd_vel.linear = Vector3(x=1.0, y=2.0)
    state.cmd_vel.angular = Vector3(z=0.5)

    state.cmd_vel_time = Mock(Time)

    return state


@pytest.fixture
def state_with_target_pose(state):
    state.move_base_goal = Mock(PoseStamped)
    state.move_base_goal.header = Header()
    point = Point(x=2.6, y=0.8, z=0.9)
    quat = Quaternion(x=0.2, y=0.3, z=0.5, w=0.8)
    state.move_base_goal.pose = Pose(position=point, orientation=quat)

    return state


@pytest.fixture
def state_with_current_pose(state):
    state.pose = Mock(PoseWithCovarianceStamped)
    state.pose.header = Header()
    state.pose.pose = pose_with_covariance()

    return state


@pytest.fixture
def state_with_gamestate(state):
    state.gamestate = Mock(GameState)
    state.gamestate.header = Header()
    state.gamestate.penalized = False

    return state


@pytest.fixture
def state():
    state = Mock(HumanoidLeagueTeamCommunication)
    state.player_id = 2
    state.team_id = own_team_id
    state.gamestate = None
    state.pose = None
    state.cmd_vel = None
    state.cmd_vel_time = None
    state.move_base_goal = None
    state.ball = None
    state.ball_velocity = None
    state.ball_covariance = None
    state.obstacles = None
    state.strategy = None
    state.strategy_time = None
    state.time_to_ball = None
    state.time_to_ball_time = None
    state.avg_walking_speed = 1.1

    return state


def relative_obstacle(obstacle_type, player_number=None):
    obstacle = Mock(ObstacleRelative)
    obstacle.type = obstacle_type
    obstacle.player_number = player_number
    obstacle.pose = PoseWithCertainty(pose=pose_with_covariance(), confidence=0.8)

    return obstacle


def pose_with_covariance():
    point = Point(x=3.6, y=1.8, z=1.9)
    quat = Quaternion(x=0.2, y=0.3, z=0.5, w=0.8)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=quat))
    pose.covariance = covariance_list()

    return pose


def covariance_list():
    covariance = numpy.empty(36, dtype=numpy.float64)
    covariance[0] = 1.0
    covariance[6] = 2.0
    covariance[30] = 3.0
    covariance[1] = 4.0
    covariance[7] = 5.0
    covariance[31] = 6.0
    covariance[5] = 7.0
    covariance[11] = 8.0
    covariance[35] = 9.0

    return covariance