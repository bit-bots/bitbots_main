import math
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time as TimeMsg
from game_controller_hsl.data import GameControlDataStruct, GameControlReturnDataStruct
from game_controller_hsl.receiver import GameStateReceiver
from game_controller_hsl_interfaces.msg import GameState, PlayerStateResponse
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion
from rclpy.clock_type import ClockType
from rclpy.time import Time
from std_msgs.msg import Header

def test_build_game_control_return_data_converts_ros_msg_to_protocol(receiver) -> None:
    receiver._clock.now.return_value = Time(seconds=12, clock_type=ClockType.ROS_TIME)

    player_pose_yaw = 0.5
    response = player_state_response(player_pose_yaw)

    packet = GameControlReturnDataStruct.parse(receiver.build_game_control_return_data(response))

    assert packet.player_number == 2
    assert packet.team_number == 6
    assert packet.fallen is True
    assert packet.pose == pytest.approx([1000.0, -2000.0, player_pose_yaw])
    assert packet.ball_age == pytest.approx(2.0)
    assert packet.ball == pytest.approx([300.0, -400.0])


def test_build_game_control_return_data_marks_unseen_ball(receiver) -> None:
    packet = GameControlReturnDataStruct.parse(
        receiver.build_game_control_return_data(PlayerStateResponse())
    )

    assert packet.ball_age == -1.0
    assert packet.pose == pytest.approx([0.0, 0.0, 0.0])
    assert packet.ball == pytest.approx([0.0, 0.0])
    assert packet.fallen is False

def test_build_game_state_msg_converts_protocol_to_ros_msg(receiver) -> None:
    packet = GameControlDataStruct.parse(game_control_data())

    msg = receiver.build_game_state_msg(packet)

    assert msg.players_per_team == 4
    assert msg.competition_type == 0
    assert msg.game_phase == GameState.GAME_PHASE_NORMAL
    assert msg.main_state == GameState.STATE_PLAYING
    assert msg.set_play == GameState.SET_PLAY_NONE
    assert msg.kicking_team == 1
    assert msg.first_half is True
    assert msg.stopped is False
    assert msg.own_score == 1
    assert msg.rival_score == 0
    assert msg.seconds_remaining == 300
    assert msg.secondary_time == 0
    assert msg.penalized is True
    assert msg.penalized_in_place is True
    assert msg.seconds_till_unpenalized == 16
    assert msg.cautions == 1
    assert msg.has_yellow_card is True
    assert msg.has_red_card is False
    assert msg.own_player_color == GameState.TEAM_RED
    assert msg.own_goalie_color == GameState.TEAM_BLACK
    assert msg.rival_player_color == GameState.TEAM_BLUE
    assert msg.rival_goalie_color == GameState.TEAM_WHITE
    assert msg.penalty_shot == 0
    assert msg.single_shots == 0b0000000000000000
    assert msg.message_budget == 10
    assert msg.team_mates_with_penalty == [True, True] + [False] * 18
    assert msg.team_mates_with_yellow_card == [False, True] + [False] * 18
    assert msg.team_mates_with_red_card == [True, False] + [False] * 18

def game_control_data() -> bytes:
    return GameControlDataStruct.build(
        dict(
            packet_number=1,
            players_per_team=4,
            competition_type=0, # COMPETITION_TYPE_SMALL
            stopped=False,
            game_phase=0, # GAME_PHASE_NORMAL
            state=3, # STATE_PLAYING
            set_play=0, # SET_PLAY_NONE
            first_half=True,
            kicking_team=1,
            secs_remaining=300,
            secondary_time=0,
            teams=[
                dict(
                    team_number=6,
                    field_player_color=1, # "TEAM_COLOR_RED
                    goalkeeper_color=3, # "TEAM_COLOR_BLACK
                    goalkeeper=1,
                    score=1,
                    penalty_shot=0,
                    single_shots=0b0000000000000000,
                    message_budget=10,
                    players=[
                        dict(
                            penalty=12, # PENALTY_SENT_OFF
                            secs_till_unpenalized=0,
                            cautions=2,
                        ),
                        dict(
                            penalty=2, # PENALTY_MOTION_IN_SET
                            secs_till_unpenalized=16,
                            cautions=1,
                        ),
                    ] + [
                        dict(
                            penalty=0, # PENALTY_NONE
                            secs_till_unpenalized=0,
                            cautions=0,
                        ) for _ in range(18)
                    ]
                ),
                dict(
                    team_number=7,
                    field_player_color=0, # TEAM_COLOR_BLUE
                    goalkeeper_color=4, # TEAM_COLOR_WHITE
                    goalkeeper=1,
                    score=0,
                    penalty_shot=0,
                    single_shots=0b0000000000000000,
                    message_budget=10,
                    players=[
                        dict(
                            penalty=0, # PENALTY_NONE
                            secs_till_unpenalized=0,
                            cautions=0,
                        ) for _ in range(20)
                    ],
                ),
            ],
        )
    )

def player_state_response(player_pose_yaw) -> PlayerStateResponse:
    return PlayerStateResponse(
        fallen=True,
        pose=PoseStamped(
            pose=Pose(
                position=Point(x=1.0, y=-2.0, z=0.0),
                orientation=Quaternion(z=math.sin(player_pose_yaw / 2.0), w=math.cos(player_pose_yaw / 2.0)),
            )
        ),
        ball=PointStamped(
            header=Header(stamp=TimeMsg(sec=10)),
            point=Point(x=0.3, y=-0.4),
        ),
    )


@pytest.fixture
def receiver() -> GameStateReceiver:
    receiver = object.__new__(GameStateReceiver)
    receiver.player_number = 2
    receiver.team_number = 6
    receiver._clock = Mock()

    return receiver
