import pytest
from humanoid_league_team_communication.converter.robocup_protocol_converter import RobocupProtocolConverter
from humanoid_league_team_communication.robocup_extension_pb2 import *
from std_msgs.msg import Header

from humanoid_league_msgs.msg import ObstacleRelative, Strategy, TeamData

own_team_id = 1


def test_convert_empty_message(snapshot, message):
    result = convert_from_message(message)

    assert str(result) == snapshot


def test_convert_current_pose(snapshot, message_with_current_pose):
    team_data = convert_from_message(message_with_current_pose)

    assert team_data.robot_id == 3
    assert str(team_data.robot_position) == snapshot


def test_convert_ball_pose(snapshot, message_with_ball):
    team_data = convert_from_message(message_with_ball)

    assert str(team_data.ball_absolute) == snapshot


def test_convert_robots_to_obstacles(snapshot, message_with_other_robots, team_data_with_header):
    team_data = convert_from_message(message_with_other_robots, team_data_with_header)

    assert team_data.obstacles.header == team_data_with_header.header
    assert team_data.obstacles.obstacles[0].type == ObstacleRelative.ROBOT_CYAN
    assert team_data.obstacles.obstacles[1].type == ObstacleRelative.ROBOT_MAGENTA

    assert str(team_data.obstacles) == snapshot


def test_convert_time_to_ball(message):
    message.time_to_ball = 16.84

    team_data = convert_from_message(message)

    assert team_data.time_to_position_at_ball == pytest.approx(16.84)


def test_convert_strategy(message_with_strategy):
    team_data = convert_from_message(message_with_strategy)

    assert team_data.strategy.role == Strategy.ROLE_STRIKER
    assert team_data.strategy.action == Strategy.ACTION_KICKING
    assert team_data.strategy.offensive_side == Strategy.SIDE_RIGHT


def convert_from_message(message: Message, team_data=TeamData()):
    return RobocupProtocolConverter().convert_from_message(message, team_data)


@pytest.fixture
def message_with_strategy(message) -> Message:
    message.role = Role.ROLE_STRIKER
    message.action = Action.ACTION_KICKING
    message.offensive_side = OffensiveSide.SIDE_RIGHT

    return message


@pytest.fixture
def message_with_other_robots(message) -> Message:
    message.others.append(robot_message(player_id=4, is_own_team=True))
    message.others.append(robot_message(player_id=2, is_own_team=False))

    return message


@pytest.fixture
def message_with_ball(message) -> Message:
    set_position(message.ball.position)
    set_covariance_matrix(message.ball.covariance)

    return message


@pytest.fixture
def message_with_current_pose(message) -> Message:
    message.current_pose.player_id = 3
    message.current_pose.team = own_team_id

    set_position(message.current_pose.position)
    set_covariance_matrix(message.current_pose.covariance)

    return message


@pytest.fixture
def team_data_with_header() -> TeamData:
    team_data = TeamData()
    team_data.header = Header()
    team_data.header.stamp.sec = 1000

    return team_data


@pytest.fixture
def message() -> Message:
    return Message()


def robot_message(player_id, is_own_team):
    robot = Robot()
    robot.player_id = player_id
    robot.team = own_team_id
    if not is_own_team:
        robot.team = own_team_id + 1

    set_position(robot.position)
    set_covariance_matrix(robot.covariance)

    return robot


def set_position(position: fvec3):
    position.x = 3.6
    position.y = 1.8
    position.z = 1.9


def set_covariance_matrix(covariance: fmat3):
    covariance.x.x = 1.2
    covariance.x.y = 2.2
    covariance.x.z = 3.2
    covariance.y.x = 4.2
    covariance.y.y = 5.2
    covariance.y.z = 6.2
    covariance.z.x = 7.2
    covariance.z.y = 8.2
    covariance.z.z = 9.2