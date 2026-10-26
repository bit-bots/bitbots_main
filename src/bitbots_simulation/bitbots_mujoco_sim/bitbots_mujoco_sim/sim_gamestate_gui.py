from dataclasses import dataclass, field

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


_TEAM_COLORS = (GameState.TEAM_BLUE, GameState.TEAM_RED)
_GOALIE_COLORS = (GameState.TEAM_YELLOW, GameState.TEAM_GREEN)
_TEAM_COLOR_NAMES = ("Blue", "Red")
_ACTIVE_STATE_COLOR = (80, 180, 80)


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


class SimGameStateGui:
    """Publishes simulated GameState messages from mjviser GUI buttons."""

    def __init__(
        self,
        node: Node,
        game_settings: list[dict[str, int | str]],
        team_sizes: TeamSizes,
        robot_topics: list[str],
    ) -> None:
        if (
            not game_settings
            or any(size < 0 for size in team_sizes)
            or len(game_settings) != len(robot_topics)
            or sum(team_sizes) < 1
            or sum(team_sizes) != len(robot_topics)
        ):
            raise ValueError("Robot topics, game settings, and team sizes must describe the same robots")

        self._node = node
        self._game = SimGame(
            kicking_team=int(game_settings[0]["team_id"]),
            scores={BASE_TEAM_ID: 0, BASE_TEAM_ID + 1: 0},
        )
        self._game_settings = game_settings
        self._team_sizes = team_sizes
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
                    self._game.main_state = state
                    self._refresh_gui()
                    self._publish()

                button.on_click(_on_game_state)

            self._team_score_labels = {}
            for team_index in range(len(_TEAM_COLOR_NAMES)):
                team_id = BASE_TEAM_ID + team_index
                score_label = server.gui.add_markdown("")
                self._team_score_labels[team_id] = score_label
                goal_button = server.gui.add_button(
                    f"Team {team_id} Goal",
                    color=DEFAULT_TEAM_COLORS[team_index],
                )

                def _on_goal(event, scoring_team_id=team_id) -> None:
                    score_goal(self._game, scoring_team_id)
                    self._refresh_gui()
                    self._publish()

                goal_button.on_click(_on_goal)

            self._refresh_gui()

    def _refresh_gui(self) -> None:
        for state, button in self._state_buttons.items():
            button.color = _ACTIVE_STATE_COLOR if state == self._game.main_state else None

        for team_id, score_label in self._team_score_labels.items():
            team_color_name = _TEAM_COLOR_NAMES[team_id - BASE_TEAM_ID]
            score_label.content = f"**Team {team_id} ({team_color_name}) score: {self._game.scores[team_id]}**"

    def _publish(self) -> None:
        stamp = self._node.get_clock().now().to_msg()
        for publisher, setting in zip(self._publishers, self._game_settings, strict=True):
            msg = build_game_state(self._game, setting, self._team_sizes)
            msg.header.stamp = stamp
            publisher.publish(msg)
