"""Only opponent kickoff positioning uses the center-circle diameter."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml  # type: ignore[import-untyped]
from bitbots_body_behavior.behavior_dsd.actions.go_to_role_position import GoToRolePosition
from game_controller_hsl_interfaces.msg import GameState


def make_blackboard(
    diameter=1.5,
    role="offense",
    position=0,
    has_kick=False,
    state=GameState.STATE_READY,
    set_play=GameState.SET_PLAY_NONE,
    phase=GameState.GAME_PHASE_NORMAL,
):
    config = yaml.safe_load((Path(__file__).parents[1] / "config/body_behavior.yaml").read_text())
    config = config["body_behavior"]["ros__parameters"]
    for name in ("offense", "defense"):
        for kickoff in ("active", "passive"):
            config["role_positions"][name][kickoff] = {
                str(key): value for key, value in config["role_positions"][name][kickoff].items()
            }
    return SimpleNamespace(
        config=config,
        gamestate=SimpleNamespace(
            has_kick=lambda: has_kick,
            get_main_state=lambda: state,
            get_set_play=lambda: set_play,
            get_game_phase=lambda: phase,
        ),
        team_data=SimpleNamespace(role=role),
        misc=SimpleNamespace(position_number=position),
        world_model=SimpleNamespace(field_length=14.0, field_width=9.0, center_circle_diameter=diameter),
    )


@pytest.mark.parametrize("diameter", [1.2, 1.5, 3.0])
@pytest.mark.parametrize("state", [GameState.STATE_READY, GameState.STATE_SET])
def test_opponent_kickoff_striker(diameter, state):
    blackboard = make_blackboard(diameter=diameter, state=state)
    blackboard.config["kickoff_striker_circle_margin"] = 0.35
    action = GoToRolePosition(blackboard, Mock(), {})
    assert action.role_position == pytest.approx([-diameter / 2 - 0.35, 0.0])


@pytest.mark.parametrize(
    "kwargs",
    [
        {"has_kick": True},
        {"role": "goalie"},
        {"role": "defense"},
        {"role": "defense", "position": 1},
        {"role": "defense", "position": 2},
        {"position": 1},
        {"position": 2},
        {"state": GameState.STATE_PLAYING},
        {"set_play": GameState.SET_PLAY_DIRECT_FREE_KICK},
        {"set_play": GameState.SET_PLAY_CORNER_KICK},
        {"phase": GameState.GAME_PHASE_PENALTY_SHOOT_OUT},
    ],
)
def test_other_positions_keep_configured_target(kwargs):
    blackboard = make_blackboard(diameter=3.0, **kwargs)
    role = blackboard.team_data.role
    target = blackboard.config["role_positions"][role]
    if role != "goalie":
        target = target["active" if blackboard.gamestate.has_kick() else "passive"][
            str(blackboard.misc.position_number)
        ]
    expected = [target[0] * blackboard.world_model.field_length / 2, target[1] * blackboard.world_model.field_width / 2]
    assert GoToRolePosition(blackboard, Mock(), {}).role_position == expected
