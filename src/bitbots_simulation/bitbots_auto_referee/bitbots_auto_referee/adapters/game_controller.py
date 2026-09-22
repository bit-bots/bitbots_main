"""Nonblocking unicast transport using the receiver's existing wire schemas."""

import math
import socket
import time
from dataclasses import asdict, dataclass

from construct import ConstructError
from game_controller_hsl.data import GameControlDataStruct, GameControlReturnDataStruct
from game_controller_hsl.data.game_control_data import MAX_NUM_PLAYERS

from bitbots_auto_referee.core.state import MatchState, PlayerState


@dataclass(frozen=True)
class RobotStatus:
    team_number: int
    player_number: int
    fallen: bool
    pose: tuple[float, float, float]
    ball_age: float
    ball: tuple[float, float]
    received_at: float


def encode_game_state(state: MatchState, packet_number: int) -> bytes:
    """Encode a complete match, padding inactive roster slots as substitutes."""
    if not 0 <= packet_number <= 255:
        raise ValueError("packet_number must fit the protocol counter")
    if not 1 <= state.players_per_team <= MAX_NUM_PLAYERS:
        raise ValueError("Invalid per-team player limit")
    if len(state.teams) != 2 or state.teams[0].team_number == state.teams[1].team_number:
        raise ValueError("A match needs distinct home and away teams")
    if state.kicking_team not in (255, *(team.team_number for team in state.teams)):
        raise ValueError("kicking_team must belong to the match or indicate no team")

    payload = asdict(state)
    teams = []
    for team in state.teams:
        if len(team.players) > state.players_per_team:
            raise ValueError("Player count exceeds the per-team limit")
        if not 1 <= team.team_number < 255:
            raise ValueError("Invalid team number")
        if not 0 <= team.goalkeeper <= state.players_per_team:
            raise ValueError("Goalkeeper must be a permitted player slot or indicate no goalkeeper")
        entry = asdict(team)
        entry["players"] = [asdict(player) for player in team.players] + [
            asdict(PlayerState(penalty="PENALTY_SUBSTITUTE")) for _ in range(MAX_NUM_PLAYERS - len(team.players))
        ]
        teams.append(entry)
    payload.update(packet_number=packet_number, teams=teams)
    return GameControlDataStruct.build(payload)


def decode_robot_status(data: bytes, received_at: float) -> RobotStatus:
    """Validate a return packet and convert lengths to meters; yaw stays in radians."""
    if len(data) != GameControlReturnDataStruct.sizeof():
        raise ValueError("Unexpected robot return packet length")
    try:
        packet = GameControlReturnDataStruct.parse(data)
    except ConstructError as error:
        raise ValueError("Invalid robot return packet header or version") from error
    if not all(math.isfinite(value) for value in (*packet.pose, packet.ball_age, *packet.ball)):
        raise ValueError("Robot return packet contains nonfinite values")
    if packet.ball_age < 0 and packet.ball_age != -1:
        raise ValueError("Invalid ball observation age")
    return RobotStatus(
        team_number=packet.team_number,
        player_number=packet.player_number,
        fallen=packet.fallen,
        pose=(packet.pose[0] / 1000.0, packet.pose[1] / 1000.0, packet.pose[2]),
        ball_age=packet.ball_age,
        ball=(packet.ball[0] / 1000.0, packet.ball[1] / 1000.0),
        received_at=received_at,
    )


class GameControllerUDPAdapter:
    """Own a return-port socket and send identical match packets to a receiver.

    Call from a single owner thread. No simulator or ROS objects are accessed here.
    The configured receiver answers the sender's IP at its configured answer_port.
    """

    def __init__(self, target: tuple[str, int], bind: tuple[str, int], state: MatchState):
        self.target = target
        self.packet_number = 0
        self.latest_responses: dict[tuple[int, int], RobotStatus] = {}
        self.rejected_packets = 0
        self.set_state(state)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.setblocking(False)
            self._socket.bind(bind)
        except OSError:
            self._socket.close()
            raise

    def set_state(self, state: MatchState) -> None:
        """Validate and atomically replace the snapshot used by the next send."""
        encode_game_state(state, self.packet_number)
        self.state = state
        self._roster = {
            (team.team_number, player_number)
            for team in state.teams
            for player_number in range(1, state.players_per_team + 1)
        }
        self.latest_responses = {key: value for key, value in self.latest_responses.items() if key in self._roster}

    def send_state(self) -> None:
        packet = encode_game_state(self.state, self.packet_number)
        self._socket.sendto(packet, self.target)
        self.packet_number = (self.packet_number + 1) % 256

    def receive_responses(self, limit: int = 64) -> list[RobotStatus]:
        """Drain a bounded batch so incoming traffic cannot starve the ROS executor."""
        responses = []
        for _ in range(limit):
            try:
                data, peer = self._socket.recvfrom(GameControlReturnDataStruct.sizeof() + 1)
            except BlockingIOError:
                break
            if peer[0] != self.target[0]:
                self.rejected_packets += 1
                continue
            try:
                response = decode_robot_status(data, time.monotonic())
            except ValueError:
                self.rejected_packets += 1
                continue
            key = (response.team_number, response.player_number)
            if key not in self._roster:
                self.rejected_packets += 1
                continue
            self.latest_responses[key] = response
            responses.append(response)
        return responses

    def close(self) -> None:
        self._socket.close()
