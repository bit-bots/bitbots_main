"""Early READY completion uses SET legality and continuous simulation-time stability."""

import math
from dataclasses import replace

import pytest

from bitbots_auto_referee.config import PARAMETERS, RefereeConfig
from bitbots_auto_referee.core.observations import RobotBounds, RobotMotion, SimulationObservation
from bitbots_auto_referee.core.state import MatchState
from bitbots_auto_referee.rules.check_rules import RuleChecker

STILL = RobotMotion(0, 0, 0, 0, 1, 1)


def setup():
    values = {name: spec.default for name, spec in PARAMETERS.items()}
    values.update(home_team_id=1, away_team_id=2)
    state = replace(MatchState.initial(RefereeConfig.from_parameters(values)), state="STATE_READY", stopped=False)
    return state, RuleChecker({0: 1, 1: 2})


def sample(time, **changes):
    positions = changes.pop("robot_positions", {0: (-2, 1, 0.4), 1: (2, 1, 0.4)})
    return replace(
        SimulationObservation(
            int(time * 1e9),
            int(time * 1000),
            (0, 0, 0.1),
            positions,
            frozenset(),
            robot_motion={robot: STILL for robot in positions},
            robot_yaws={robot: 0.0 for robot in positions},
            robot_bounds={
                robot: RobotBounds(x - 0.1, x + 0.1, y - 0.1, y + 0.1)
                for robot, (x, y, _) in positions.items()
            },
        ),
        **changes,
    )


def test_legal_stationary_lineup_advances_after_continuous_stability():
    state, checker = setup()
    for time in (0, 4.999):
        state = checker.check_rules(state, sample(time))
        assert state.state == "STATE_READY"
    state = checker.check_rules(state, sample(5))
    assert state.state == "STATE_SET" and state.secondary_time == 5 and not state.stopped


@pytest.mark.parametrize(
    "change",
    [
        {"robot_motion": {0: replace(STILL, linear_speed=0.1), 1: STILL}},
        {"robot_motion": {0: replace(STILL, angular_speed=0.5), 1: STILL}},
        {"robot_motion": {0: replace(STILL, body_joint_speed=1.0), 1: STILL}},
        {"robot_yaws": {0: 0.5, 1: 0.0}},
        {"teleported_robots": frozenset({0})},
        {"robot_motion": {}},
        {"robot_yaws": {0: float("nan"), 1: 0.0}},
        {"robot_bounds": {}},
    ],
)
def test_motion_teleports_and_missing_data_restart_ready_stability(change):
    state, checker = setup()
    state = checker.check_rules(state, sample(0))
    state = checker.check_rules(state, sample(4, **change))
    state = checker.check_rules(state, sample(5))
    assert state.state == "STATE_READY"
    state = checker.check_rules(state, sample(10))
    assert state.state == "STATE_SET"


def test_pose_drift_accumulates_against_stability_anchor():
    state, checker = setup()
    state = checker.check_rules(state, sample(0))
    state = checker.check_rules(state, sample(3, robot_positions={0: (-1.96, 1, 0.4), 1: (2, 1, 0.4)}))
    state = checker.check_rules(state, sample(5, robot_positions={0: (-1.96, 1, 0.4), 1: (2, 1, 0.4)}))
    assert state.state == "STATE_READY"
    state = checker.check_rules(state, sample(8, robot_positions={0: (-1.96, 1, 0.4), 1: (2, 1, 0.4)}))
    assert state.state == "STATE_SET"


def test_yaw_wrap_is_not_interpreted_as_rotation():
    state, checker = setup()
    state = checker.check_rules(state, sample(0, robot_yaws={0: math.pi - 0.01, 1: 0}))
    state = checker.check_rules(state, sample(5, robot_yaws={0: -math.pi + 0.01, 1: 0}))
    assert state.state == "STATE_SET"


def test_illegal_or_missing_player_prevents_early_transition():
    for positions in ({0: (2, 1, 0.4), 1: (2, -1, 0.4)}, {0: (-2, 1, 0.4)}):
        state, checker = setup()
        for time in (0, 5):
            state = checker.check_rules(state, sample(time, robot_positions=positions))
        assert state.state == "STATE_READY"


def test_penalized_sideline_robot_does_not_block_ready():
    state, checker = setup()
    home, away = state.teams
    away = replace(
        away, players=(replace(away.players[0], penalty="PENALTY_PUSHING", secs_till_unpenalized=30), *away.players[1:])
    )
    state = replace(state, teams=(home, away))
    for time in (0, 5):
        data = sample(time, robot_positions={0: (-2, 1, 0.4), 1: (2, 3.3, 0.4)})
        data = replace(data, robot_motion={0: STILL, 1: replace(STILL, angular_speed=2.0)})
        state = checker.check_rules(state, data)
    assert state.state == "STATE_SET"
    assert state.teams[1].players[0].penalty == "PENALTY_PUSHING"


@pytest.mark.parametrize("owner", ["goal", "neutral"])
def test_restart_sequences_preserve_full_set_after_early_ready(owner):
    state, checker = setup()
    if owner == "goal":
        checker._restart_at = 10_000_000_000
        checker._restart_is_goal = True
    else:
        state = replace(state, kicking_team=255)
        checker.game_flow_rules._restart_at = 10_000_000_000
    for time in (10, 15):
        state = checker.check_rules(state, sample(time))
    assert state.state == "STATE_SET" and state.secondary_time == 5
    state = checker.check_rules(state, sample(19.999))
    assert state.state == "STATE_SET"
    state = checker.check_rules(state, sample(20))
    assert state.state == "STATE_PLAYING"


def test_simulation_reset_discards_previous_ready_stability():
    state, checker = setup()
    for time in (10, 0, 4):
        state = checker.check_rules(state, sample(time))
        assert state.state == "STATE_READY"
    state = checker.check_rules(state, sample(5))
    assert state.state == "STATE_SET"


def test_ready_uses_the_same_single_kicker_exception_as_set():
    state, checker = setup()
    for time in (0, 5):
        state = checker.check_rules(state, sample(time, robot_positions={0: (0.2, 0, 0.4), 1: (2, 1, 0.4)}))
    assert state.state == "STATE_SET"
    state, _ = setup()
    checker = RuleChecker({0: 1, 1: 1, 2: 2})
    for time in (0, 5):
        state = checker.check_rules(
            state, sample(time, robot_positions={0: (0.2, 0, 0.4), 1: (-0.2, 0, 0.4), 2: (2, 1, 0.4)})
        )
    assert state.state == "STATE_READY"
