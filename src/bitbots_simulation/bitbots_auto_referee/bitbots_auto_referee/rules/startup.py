"""Clock-driven opening sequence, independent of ROS and robot connectivity."""

from dataclasses import replace

from bitbots_auto_referee.core.state import MatchState

NANOSECONDS_PER_SECOND = 1_000_000_000
STARTUP_PHASES = (
    ("STATE_INITIAL", 5),
    ("STATE_READY", 25),
    ("STATE_SET", 5),
)

PHASE_SECONDS = dict(STARTUP_PHASES)


class StartupSequence:
    """Advance once to PLAYING; restart unfinished preparation on a clock reset."""

    def __init__(self):
        self._started_at: int | None = None
        self._last_time: int | None = None
        self.finished = False

    def finish_ready(self, now_ns: int) -> None:
        """Skip the remaining READY time while preserving the complete SET interval."""
        if not self.finished and self._started_at is not None:
            before_set = PHASE_SECONDS["STATE_INITIAL"] + PHASE_SECONDS["STATE_READY"]
            self._started_at = now_ns - before_set * NANOSECONDS_PER_SECOND
            self._last_time = now_ns

    def advance(self, state: MatchState, now_ns: int) -> MatchState:
        """Use elapsed time since the first clock sample, preserving other match fields."""
        if self.finished:
            return state
        if self._started_at is None or (self._last_time is not None and now_ns < self._last_time):
            self._started_at = now_ns
        self._last_time = now_ns
        elapsed = now_ns - self._started_at
        boundary = 0
        for phase, duration in STARTUP_PHASES:
            boundary += duration * NANOSECONDS_PER_SECOND
            if elapsed < boundary:
                remaining = (boundary - elapsed + NANOSECONDS_PER_SECOND - 1) // NANOSECONDS_PER_SECOND
                return replace(
                    state,
                    state=phase,
                    stopped=phase == "STATE_INITIAL",
                    secondary_time=0 if phase == "STATE_INITIAL" else remaining,
                )
        self.finished = True
        return replace(state, state="STATE_PLAYING", stopped=False, secondary_time=0)
