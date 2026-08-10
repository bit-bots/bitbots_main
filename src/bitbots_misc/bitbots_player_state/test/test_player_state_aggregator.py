from unittest.mock import Mock

import pytest
from bitbots_player_state.player_state_aggregator import PlayerStateAggregator
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Header

from bitbots_msgs.msg import RobotControlState


def test_build_response_aggregates_latest_state(aggregator):
    pose = PoseWithCovarianceStamped(
        header=Header(stamp=Time(sec=10), frame_id="map"),
    )
    pose.pose.pose.position.x = 1.0
    pose.pose.pose.position.y = -2.0
    pose.pose.pose.orientation = Quaternion(w=1.0)
    ball = PoseWithCovarianceStamped(
        header=Header(stamp=Time(sec=11), frame_id="odom"),
    )
    ball.pose.pose.position = Point(x=0.4, y=-0.2)

    transformed_ball = PointStamped(
        header=Header(stamp=Time(sec=11), frame_id="base_footprint"),
        point=Point(x=0.1, y=0.3),
    )
    aggregator._tf_buffer.transform.return_value = transformed_ball

    aggregator._pose_callback(pose)
    aggregator._robot_state_callback(RobotControlState(state=RobotControlState.FALLEN))
    aggregator._ball_callback(ball)

    response = aggregator.build_response()

    aggregator._tf_buffer.transform.assert_called_once_with(aggregator._relative_ball, "base_footprint")
    assert aggregator._relative_ball.header == Header(stamp=Time(sec=11), frame_id="odom")
    assert response.header == Header(stamp=Time(sec=12), frame_id="")
    assert response.fallen is True
    assert response.pose.header == Header(stamp=Time(sec=10), frame_id="map")
    assert response.pose.pose.position == Point(x=1.0, y=-2.0)
    assert response.ball == transformed_ball


@pytest.fixture
def aggregator():
    node = object.__new__(PlayerStateAggregator)
    node._fallen = False
    node._pose = None
    node._relative_ball = None
    node._tf_buffer = Mock()
    node.ball_frame_out = "base_footprint"
    node._clock = Mock()
    node._clock.now.return_value.to_msg.return_value = Time(sec=12)
    return node
