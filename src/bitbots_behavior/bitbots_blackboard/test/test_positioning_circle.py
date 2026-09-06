"""Set-play positioning follows the active field geometry."""

import numpy as np
import pytest
from bitbots_blackboard.capsules.positioning_capsule import Field, InnerPositioningCapsule, Params, Role


@pytest.mark.parametrize("diameter", [1.3, 1.5, 3.0])
def test_striker_respects_center_circle(diameter):
    field = Field(length=14.0, width=9.0, center_circle_diameter=diameter)
    params = Params()
    formation = InnerPositioningCapsule()._compute_formation(np.zeros(2), field, 4, params, opp_set_play=True)
    striker = formation[Role.STRIKER]
    assert np.linalg.norm(striker[:2]) >= diameter / 2
    assert np.linalg.norm(striker[:2]) >= params.opp_set_play_clearance
