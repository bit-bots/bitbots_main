"""Team send accounting and reachability for the configured robot identities."""

from dataclasses import replace

from bitbots_auto_referee.core.state import MatchState


class MessageBudget:
    def __init__(self):
        self._counts: dict[tuple[int, int, str], int] = {}

    def update(self, state: MatchState, team_id: int, player_id: int, session: str, sent: int) -> MatchState:
        if not isinstance(session, str) or not session or type(sent) is not int or sent < 0:
            raise ValueError("Send report requires a session and a nonnegative cumulative count")
        if team_id not in {team.team_number for team in state.teams}:
            raise ValueError("Send report is outside the match")
        key = (team_id, player_id, session)
        previous = self._counts.get(key, 0)
        if sent <= previous:
            return state
        self._counts[key] = sent
        teams = tuple(
            replace(team, message_budget=max(0, team.message_budget - (sent - previous)))
            if team.team_number == team_id
            else team
            for team in state.teams
        )
        return replace(state, teams=teams)


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
