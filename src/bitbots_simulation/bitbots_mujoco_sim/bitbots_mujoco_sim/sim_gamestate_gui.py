import select
import socket
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from game_controller_hsl_interfaces.msg import GameState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from bitbots_mujoco_sim.world import BASE_TEAM_ID, DEFAULT_TEAM_COLORS, TeamSizes

_GAME_STATES = (
    ("INITIAL", GameState.STATE_INITIAL),
    ("READY", GameState.STATE_READY),
    ("SET", GameState.STATE_SET),
    ("PLAYING", GameState.STATE_PLAYING),
    ("FINISHED", GameState.STATE_FINISHED),
)


@dataclass
class SimGame:
    main_state: int = GameState.STATE_INITIAL
    kicking_team: int = BASE_TEAM_ID
    scores: dict[int, int] = field(default_factory=dict)
    message_budgets: dict[int, int] = field(default_factory=dict)
    sent_messages: dict[int, int] = field(default_factory=dict)


_TEAM_COLORS = (GameState.TEAM_BLUE, GameState.TEAM_RED)
_GOALIE_COLORS = (GameState.TEAM_YELLOW, GameState.TEAM_GREEN)
_TEAM_COLOR_NAMES = ("Blue", "Red")
_ACTIVE_STATE_COLOR = (80, 180, 80)
_TEAM_IDS = (BASE_TEAM_ID, BASE_TEAM_ID + 1)
_MAX_MESSAGE_BUDGET = 65535


def build_game_state(game: SimGame, robot_setting: dict[str, int | str], team_sizes: TeamSizes) -> GameState:
    """Build a GameState from the perspective of one robot's team."""
    if any(size < 0 for size in team_sizes) or sum(team_sizes) < 1:
        raise ValueError("GAME_STATE requires a non-empty two-team setup")

    team_id = int(robot_setting["team_id"])
    player_number = int(robot_setting["bot_id"])
    team_color = int(robot_setting["team_color"])
    if team_color < 0 or team_color >= len(_TEAM_COLORS):
        raise ValueError(f"Team color index {team_color} is invalid")
    team_index = team_id - BASE_TEAM_ID
    if team_index not in (0, 1):
        raise ValueError(f"Team ID {team_id} is not present in the game setup")
    if player_number < 1 or player_number > team_sizes[team_index]:
        raise ValueError(f"Player number {player_number} is invalid for team {team_id}")

    rival_team_index = 1 - team_index
    rival_team_id = BASE_TEAM_ID + rival_team_index
    rival_color = rival_team_index

    if game.kicking_team not in (team_id, rival_team_id):
        raise ValueError(f"Kicking team {game.kicking_team} is not part of this game")

    msg = GameState()
    msg.players_per_team = max(team_sizes)
    msg.main_state = game.main_state
    msg.kicking_team = game.kicking_team
    msg.own_score = game.scores[team_id]
    msg.rival_score = game.scores[rival_team_id]
    if team_id not in game.message_budgets:
        raise ValueError(f"No message budget is configured for team {team_id}")
    msg.message_budget = game.message_budgets[team_id]
    msg.own_player_color = _TEAM_COLORS[team_color]
    msg.own_goalie_color = _GOALIE_COLORS[team_color]
    msg.rival_player_color = _TEAM_COLORS[rival_color]
    msg.rival_goalie_color = _GOALIE_COLORS[rival_color]

    player_count = max(msg.players_per_team, player_number)
    msg.team_mates_with_penalty = [False] * player_count
    msg.team_mates_with_yellow_card = [False] * player_count
    msg.team_mates_with_red_card = [False] * player_count
    return msg


def score_goal(game: SimGame, scoring_team_id: int) -> None:
    """Record a goal and reset the game to READY for the conceding team."""
    if scoring_team_id not in (BASE_TEAM_ID, BASE_TEAM_ID + 1):
        raise ValueError(f"Team ID {scoring_team_id} is not part of this game")

    conceding_team_id = BASE_TEAM_ID + 1 if scoring_team_id == BASE_TEAM_ID else BASE_TEAM_ID
    if scoring_team_id not in game.scores or conceding_team_id not in game.scores:
        raise ValueError("Scores must be initialized for both game teams")

    game.scores[scoring_team_id] += 1
    game.main_state = GameState.STATE_READY
    game.kicking_team = conceding_team_id


def consume_message_budget(game: SimGame, team_id: int, count: int) -> None:
    """Account for observed sent messages and decrement the team's remaining budget."""
    if team_id not in game.message_budgets:
        raise ValueError(f"Team ID {team_id} has no message budget")
    if count < 0:
        raise ValueError("Message count cannot be negative")

    game.sent_messages[team_id] = game.sent_messages.get(team_id, 0) + count
    game.message_budgets[team_id] = max(0, game.message_budgets[team_id] - count)


class TeamCommBudgetListener:
    """Counts team communication datagrams sent to the simulator's reserved ports."""

    def __init__(self, logger: Any, base_port: int, on_messages: Callable[[int, int], None]) -> None:
        self._logger = logger
        self._on_messages = on_messages
        self._stop_event = threading.Event()
        self._sockets: dict[int, socket.socket] = {}
        self._thread: threading.Thread | None = None

        last_port = base_port + 10 * _TEAM_IDS[-1]
        if base_port < 1 or last_port > 65535:
            self._logger.error(
                f"Invalid team communication budget base port {base_port}; message budget tracking is disabled"
            )
            return

        try:
            for team_id in _TEAM_IDS:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
                self._sockets[team_id] = sock
                sock.bind(("0.0.0.0", base_port + 10 * team_id))
                sock.setblocking(False)
        except OSError as error:
            self._close_sockets()
            self._logger.error(
                f"Could not bind team communication budget port: {error}; message budget tracking is disabled"
            )
            return

        self._logger.info(
            f"Listening for team communication budget packets on ports "
            f"{base_port + 10 * _TEAM_IDS[0]} and {base_port + 10 * _TEAM_IDS[1]}"
        )
        self._thread = threading.Thread(target=self._listen, name="team-comm-budget-listener", daemon=True)
        self._thread.start()

    @property
    def enabled(self) -> bool:
        return self._thread is not None

    def _listen(self) -> None:
        pending_counts = {team_id: 0 for team_id in _TEAM_IDS}
        socket_to_team = {sock: team_id for team_id, sock in self._sockets.items()}
        last_flush = time.monotonic()

        while not self._stop_event.is_set():
            try:
                readable, _, _ = select.select(list(socket_to_team), [], [], 0.2)
            except OSError as error:
                if not self._stop_event.is_set():
                    self._logger.error(f"Team communication budget listener stopped: {error}")
                break

            for sock in readable:
                team_id = socket_to_team[sock]
                while True:
                    try:
                        sock.recvfrom(65535)
                    except BlockingIOError:
                        break
                    except OSError as error:
                        self._logger.error(f"Could not receive team {team_id} communication packet: {error}")
                        break
                    pending_counts[team_id] += 1

            now = time.monotonic()
            if now - last_flush >= 1.0:
                self._flush_counts(pending_counts)
                last_flush = now

        self._flush_counts(pending_counts)

    def _flush_counts(self, pending_counts: dict[int, int]) -> None:
        for team_id, count in pending_counts.items():
            if count:
                try:
                    self._on_messages(team_id, count)
                except Exception as error:
                    self._logger.error(f"Could not update team {team_id} message budget: {error}")
                else:
                    pending_counts[team_id] = 0

    def _close_sockets(self) -> None:
        for sock in self._sockets.values():
            sock.close()
        self._sockets.clear()

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._close_sockets()


class SimGameStateGui:
    """Publishes simulated GameState messages from mjviser GUI buttons."""

    def __init__(
        self,
        node: Node,
        game_settings: list[dict[str, int | str]],
        team_sizes: TeamSizes,
        robot_topics: list[str],
        message_budget: int,
        team_comm_budget_port: int,
    ) -> None:
        if (
            not game_settings
            or any(size < 0 for size in team_sizes)
            or len(game_settings) != len(robot_topics)
            or sum(team_sizes) < 1
            or sum(team_sizes) != len(robot_topics)
        ):
            raise ValueError("Robot topics, game settings, and team sizes must describe the same robots")
        if not 0 <= message_budget <= _MAX_MESSAGE_BUDGET:
            raise ValueError(f"Initial message budget must be between 0 and {_MAX_MESSAGE_BUDGET}")

        self._node = node
        self._game = SimGame(
            kicking_team=int(game_settings[0]["team_id"]),
            scores={BASE_TEAM_ID: 0, BASE_TEAM_ID + 1: 0},
            message_budgets={team_id: message_budget for team_id in _TEAM_IDS},
            sent_messages={team_id: 0 for team_id in _TEAM_IDS},
        )
        self._game_settings = game_settings
        self._team_sizes = team_sizes
        self._team_comm_budget_port = team_comm_budget_port
        self._game_lock = threading.Lock()
        self._budget_listener: TeamCommBudgetListener | None = None
        qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self._publishers = [node.create_publisher(GameState, topic, qos) for topic in robot_topics]

    def add_to_viewer(self, server) -> None:
        tabs = server.gui.add_tab_group()
        with tabs.add_tab("Game State"):
            self._state_buttons = {}
            for label, value in _GAME_STATES:
                button = server.gui.add_button(label)
                self._state_buttons[value] = button

                def _on_game_state(event, state=value) -> None:
                    with self._game_lock:
                        self._game.main_state = state
                        self._refresh_gui()
                        self._publish()

                button.on_click(_on_game_state)

            self._team_score_labels = {}
            self._budget_status = server.gui.add_markdown("")
            for team_index in range(len(_TEAM_COLOR_NAMES)):
                team_id = BASE_TEAM_ID + team_index
                score_label = server.gui.add_markdown("")
                self._team_score_labels[team_id] = score_label
                goal_button = server.gui.add_button(
                    f"Team {team_id} Goal",
                    color=DEFAULT_TEAM_COLORS[team_index],
                )

                def _on_goal(event, scoring_team_id=team_id) -> None:
                    with self._game_lock:
                        score_goal(self._game, scoring_team_id)
                        self._refresh_gui()
                        self._publish()

                goal_button.on_click(_on_goal)

            self._refresh_gui()
            self._budget_listener = TeamCommBudgetListener(
                self._node.get_logger(), self._team_comm_budget_port, self._on_messages_received
            )
            self._budget_status.content = (
                "Team communication packet tracking is active."
                if self._budget_listener.enabled
                else "**Team communication packet tracking is disabled; see simulator logs.**"
            )

    def _refresh_gui(self) -> None:
        for state, button in self._state_buttons.items():
            button.color = _ACTIVE_STATE_COLOR if state == self._game.main_state else None

        for team_id, score_label in self._team_score_labels.items():
            team_color_name = _TEAM_COLOR_NAMES[team_id - BASE_TEAM_ID]
            score_label.content = (
                f"**Team {team_id} ({team_color_name}) score: {self._game.scores[team_id]}**  \n"
                f"Messages sent: {self._game.sent_messages[team_id]}  \n"
                f"Message budget remaining: {self._game.message_budgets[team_id]}"
            )

    def _on_messages_received(self, team_id: int, count: int) -> None:
        with self._game_lock:
            consume_message_budget(self._game, team_id, count)
            self._refresh_gui()
            self._publish()

    def close(self) -> None:
        if self._budget_listener is not None:
            self._budget_listener.close()
            self._budget_listener = None

    def _publish(self) -> None:
        stamp = self._node.get_clock().now().to_msg()
        for publisher, setting in zip(self._publishers, self._game_settings, strict=True):
            msg = build_game_state(self._game, setting, self._team_sizes)
            msg.header.stamp = stamp
            publisher.publish(msg)
