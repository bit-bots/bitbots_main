import subprocess
import sys
from pathlib import Path

import bitbots_team_communication.robocup_extension_pb2 as ExtendedProto  # noqa: N812
from bitbots_msgs.msg import RobotRelative, Strategy, TeamData
from bitbots_team_communication.converter.robocup_protocol_converter import RobocupProtocolConverter, TeamColor

PROTOCOL_DIR = Path(__file__).parents[2] / "bitbots_team_communication" / "RobocupProtocol"
BASE_PROTO = PROTOCOL_DIR / "robocup.proto"


def test_extended_protocol_handles_message_without_extensions(tmp_path: Path):
    serialized_message = serialize_message_without_extensions(tmp_path)

    message = ExtendedProto.Message()
    message.ParseFromString(serialized_message)
    team_data = RobocupProtocolConverter(TeamColor.BLUE).convert_from_message(message, TeamData())

    assert message.current_pose.player_id == 3
    assert message.current_pose.position.x == 1.25
    assert message.current_pose.position.y == -0.5
    assert message.current_pose.team == ExtendedProto.BLUE
    assert not message.other_robot_confidence

    assert team_data.robot_id == 3
    assert team_data.robot_position.pose.position.x == 1.25
    assert team_data.robot_position.pose.position.y == -0.5
    assert team_data.robots.robots[0].player_number == 5
    assert team_data.robots.robots[0].pose.pose.pose.position.x == -1.0
    assert team_data.robots.robots[0].type == RobotRelative.ROBOT_RED

    # Fields unknown to the sender retain their enum or scalar default values.
    assert team_data.time_to_position_at_ball == 0.0
    assert team_data.strategy.role == Strategy.ROLE_UNDEFINED
    assert team_data.strategy.action == Strategy.ACTION_UNDEFINED
    assert team_data.strategy.offensive_side == Strategy.SIDE_UNDEFINED


def serialize_message_without_extensions(output_dir: Path) -> bytes:
    """Generate the base bindings and serialize with them in an isolated process.

    The base and extended schemas use identical fully-qualified message names, so
    loading both generated modules into one process would cause a descriptor-pool
    collision. The subprocess also proves that the payload originates from the
    extension-free schema rather than from the receiver's extended bindings.
    """
    subprocess.run(
        [
            "protoc",
            f"--proto_path={PROTOCOL_DIR}",
            f"--python_out={output_dir}",
            str(BASE_PROTO),
        ],
        check=True,
    )

    encoder = """
import sys

import robocup_pb2 as Proto

message = Proto.Message()
message.timestamp.seconds = 123
message.state = Proto.UNPENALISED
message.current_pose.player_id = 3
message.current_pose.position.x = 1.25
message.current_pose.position.y = -0.5
message.current_pose.position.z = 0.75
message.current_pose.team = Proto.BLUE
message.ball.position.x = 2.0
message.ball.position.y = 0.25

opponent = message.others.add()
opponent.player_id = 5
opponent.position.x = -1.0
opponent.team = Proto.RED

sys.stdout.buffer.write(message.SerializeToString())
"""
    result = subprocess.run(
        [sys.executable, "-c", encoder],
        cwd=output_dir,
        check=True,
        capture_output=True,
    )
    return result.stdout
