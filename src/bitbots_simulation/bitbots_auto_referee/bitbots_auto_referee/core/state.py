"""Immutable match snapshots shared with the GameController adapter."""

from dataclasses import dataclass

from bitbots_auto_referee.config import RefereeConfig


@dataclass(frozen=True)
class PlayerState:
    penalty: str = "PENALTY_NONE"
    secs_till_unpenalized: int = 0
    cautions: int = 0


@dataclass(frozen=True)
class TeamState:
    """Player slots ordered by protocol player number, independent of connectivity."""

    team_number: int
    field_player_color: str
    goalkeeper_color: str
    players: tuple[PlayerState, ...]
    goalkeeper: int = 1
    score: int = 0
    penalty_shot: int = 0
    single_shots: int = 0
    message_budget: int = 0


@dataclass(frozen=True)
class MatchState:
    """Match snapshot whose players_per_team is an upper limit, not a required count."""

    competition_type: str
    players_per_team: int
    teams: tuple[TeamState, TeamState]
    secs_remaining: int = 600
    state: str = "STATE_INITIAL"
    stopped: bool = True
    game_phase: str = "GAME_PHASE_NORMAL"
    set_play: str = "SET_PLAY_NONE"
    first_half: bool = True
    kicking_team: int = 255
    secondary_time: int = 0

    @classmethod
    def initial(cls, config: RefereeConfig) -> "MatchState":
        """Prepare permitted player slots without assuming any robot is connected."""

        def team(team_id: int, color: str, goalkeeper_color: str) -> TeamState:
            return TeamState(
                team_number=team_id,
                field_player_color=color.upper(),
                goalkeeper_color=goalkeeper_color.upper(),
                players=tuple(PlayerState() for _ in range(config.players_per_team)),
            )

        return cls(
            competition_type=f"COMPETITION_TYPE_{config.league_size.upper()}",
            players_per_team=config.players_per_team,
            teams=(
                team(config.home_team_id, config.home_color, config.home_goalkeeper_color),
                team(config.away_team_id, config.away_color, config.away_goalkeeper_color),
            ),
            kicking_team=config.home_team_id,
        )
