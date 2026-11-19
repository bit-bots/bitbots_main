"""Referee snapshots refer to the contact solve rather than integrated coordinates."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

import mujoco
import numpy as np
import pytest
from bitbots_mujoco_sim.referee import RefereeObservationBuilder, RefereeStepState
from bitbots_mujoco_sim.simulation import Simulation

from bitbots_msgs.msg import SimulationState


@pytest.mark.parametrize("integrator", ["Euler", "implicit", "implicitfast"])
def test_solved_contact_matches_saved_positions_velocities_and_bounds(integrator):
    model = mujoco.MjModel.from_xml_string(f"""
        <mujoco><option gravity="0 0 0" timestep="0.01" integrator="{integrator}"/>
          <worldbody>
            <body name="robot" pos="0 0 1"><freejoint/><geom size="0.12"/></body>
            <body name="ball" pos="0.22 0 1"><freejoint/><geom size="0.12"/></body>
          </worldbody>
        </mujoco>
    """)
    data = mujoco.MjData(model)
    data.qvel[0] = 2.0
    data.qvel[6] = -1.0
    builder = RefereeObservationBuilder(model, {0: model.body("robot").id}, 1)
    state = RefereeStepState.capture(data)
    mujoco.mj_step(model, data)
    message = builder.build(data, SimulationState().header.stamp, 1, state=state)
    robot = message.robots[0]
    assert data.qpos[0] != pytest.approx(state.qpos[0])
    assert data.qvel[0] != pytest.approx(state.qvel[0])
    assert robot.position.x == state.qpos[0]
    assert message.ball_position.x == state.qpos[7]
    assert robot.linear_speed == 2.0
    assert robot.upright == 1.0
    assert robot.min_x == pytest.approx(state.qpos[0] - 0.12)
    assert robot.max_x == pytest.approx(state.qpos[0] + 0.12)
    assert robot.touching_ball and robot.ball_contact_force > 0


def test_rk4_cannot_silently_produce_inconsistent_referee_observations():
    model = mujoco.MjModel.from_xml_string('<mujoco><option integrator="RK4"/></mujoco>')
    with pytest.raises(ValueError, match="single-step integrator"):
        RefereeObservationBuilder(model, {}, -1)


def test_step_captures_after_teleports_and_before_integration():
    simulation = Mock()
    simulation.data = SimpleNamespace(time=1.25, qpos=np.zeros(7), qvel=np.zeros(6))
    simulation.step_number = 5
    simulation.time = 1.25
    simulation.timestep = 0.01
    simulation.early_events = []
    simulation.events = []

    def teleport(data, step):
        data.qpos[0] = 4.0
        return {0}, False

    def integrate(model, data):
        data.qpos[0] = 5.0
        data.qvel[0] = 2.0
        data.time += simulation.timestep

    simulation.teleports.apply_pending.side_effect = teleport
    with patch("bitbots_mujoco_sim.simulation.mujoco.mj_step", side_effect=integrate):
        Simulation.step(simulation)
    simulation.teleports.apply_pending.assert_called_once_with(simulation.data, 6)
    state = simulation.publish_referee_step.call_args.args[0]
    assert state.time == 1.25
    assert state.qpos[0] == 4.0 and state.qvel[0] == 0.0
    assert simulation.data.qpos[0] == 5.0 and simulation.data.qvel[0] == 2.0
    assert simulation._teleported_robots == {0}

    Simulation.publish_referee_step(simulation, state)
    args, kwargs = simulation.referee_observations.build.call_args
    assert args[1].sec == 1 and args[1].nanosec == 250_000_000
    assert args[2:] == (6, {0}, False)
    assert kwargs["state"] is state
    simulation.referee_step_publisher.publish.assert_called_once_with(
        simulation.referee_observations.build.return_value
    )
