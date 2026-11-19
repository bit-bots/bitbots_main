"""Team send accounting and reachability for the configured robot identities."""

from dataclasses import replace

from bitbots_auto_referee.core.state import MatchState


def count_team_messages(state: MatchState, counts: dict[int, int]) -> MatchState:
    """Deduct received UDP datagrams from each team's shared budget."""
    teams = tuple(
        replace(team, message_budget=max(0, team.message_budget - counts.get(team.team_number, 0)))
        for team in state.teams
    )
    return replace(state, teams=teams) if teams != state.teams else state


def robot_connections(
    expected: set[tuple[int, int]], responses: dict, now: float, started_at: float, timeout: float
) -> dict[tuple[int, int], bool | None]:
    """Use wall time so paused physics cannot hide a lost receiver."""
    result: dict[tuple[int, int], bool | None] = {}
    for identity in sorted(expected):
        response = responses.get(identity)
        if response is not None:
            result[identity] = now - response.received_at < timeout
        else:
            result[identity] = None if now - started_at < timeout else False
    return result
