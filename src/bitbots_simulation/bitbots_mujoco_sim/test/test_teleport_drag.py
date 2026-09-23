"""A referee teleport must release the ball from an unfinished viewer drag."""

import asyncio
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from bitbots_mujoco_sim.simulation import Simulation


def test_teleport_releases_drag_and_rejects_late_mouse_updates():
    simulation = SimpleNamespace(
        model=SimpleNamespace(jnt_qposadr=[0], jnt_dofadr=[0]),
        data=SimpleNamespace(qpos=np.zeros(7), qvel=np.zeros(6)),
        teleports=SimpleNamespace(robot_joints={}),
        ball_joint_id=0,
        _teleported_robots=set(),
        _ball_teleported=True,
        _drag_versions={},
        _drag_lock=Lock(),
        _drag_targets={},
        _rotation_targets={},
    )
    handle = Mock()
    Simulation._register_free_joint_drag(simulation, handle, lambda event: (0, 0))
    callback = handle.on_drag.call_args.args[0]

    def drag(phase):
        asyncio.run(callback(SimpleNamespace(phase=phase, start_position=(0, 0, 0), end_position=(1, 0, 0))))

    drag("start")
    drag("update")
    assert (0, 0) in simulation._drag_targets
    Simulation._release_teleported_drag_targets(simulation)
    simulation.data.qpos[0] = 2.0
    simulation.data.qvel[0] = 0.5
    drag("update")
    Simulation._apply_drag_targets(simulation)
    assert not simulation._drag_targets
    assert simulation.data.qpos[0] == 2.0
    assert simulation.data.qvel[0] == 0.5
    drag("start")
    drag("update")
    assert (0, 0) in simulation._drag_targets
