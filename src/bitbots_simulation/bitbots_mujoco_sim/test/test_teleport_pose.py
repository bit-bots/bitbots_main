"""Planar teleports preserve tilted robot poses and their vertical geometry."""

import math

import mujoco
import numpy as np
import pytest
from bitbots_mujoco_sim.teleport import TeleportController
from rclpy.task import Future
from transforms3d.euler import euler2quat, mat2euler

from bitbots_msgs.srv import Teleport


@pytest.mark.parametrize("roll,pitch", [(0.0, 0.2), (1.3, 0.2), (math.pi / 2, 0.2), (0.0, math.pi / 2)])
def test_yaw_teleport_preserves_height_tilt_and_articulated_pose(roll, pitch):
    model = mujoco.MjModel.from_xml_string("""
        <mujoco><worldbody>
          <body name="robot" pos="0 0 0.15"><freejoint/>
            <geom type="box" size="0.2 0.1 0.05"/>
            <body pos="0.25 0 0"><joint name="limb"/>
              <geom type="box" size="0.05 0.03 0.02"/>
            </body>
          </body>
        </worldbody><actuator><position joint="limb" kp="1"/></actuator></mujoco>
    """)
    data = mujoco.MjData(model)
    data.qpos[3:7] = euler2quat(roll, pitch, 0.4)
    data.qpos[7] = 0.3
    data.qvel[:] = 0.6
    data.qacc_warmstart[:] = 0.4
    data.ctrl[:] = 0.3
    mujoco.mj_forward(model, data)
    height = data.qpos[2]
    vertical_centers = data.geom_xpos[:, 2].copy()
    vertical_rotations = data.geom_xmat.reshape(-1, 3, 3)[:, 2, :].copy()
    controls = data.ctrl.copy()
    controller = TeleportController(model, {0: model.body("robot").id}, -1)
    completed = Future()
    request = Teleport.Request(target_type=Teleport.Request.ROBOT, robot_index=0, x=2.0, y=-3.0, yaw=-0.8)
    controller._queue.put_nowait((request, controller.robot_joints[0], completed))
    robots, ball = controller.apply_pending(data, 12)
    assert robots == {0} and not ball
    assert completed.result().success and completed.result().applied_step == 12
    np.testing.assert_allclose(data.qpos[:3], [2.0, -3.0, height])
    assert data.qpos[7] == 0.3
    if abs(math.cos(pitch)) > 1e-6:
        assert mat2euler(data.xmat[model.body("robot").id].reshape(3, 3))[2] == pytest.approx(-0.8)
    np.testing.assert_allclose(data.geom_xpos[:, 2], vertical_centers, atol=1e-12)
    np.testing.assert_allclose(data.geom_xmat.reshape(-1, 3, 3)[:, 2, :], vertical_rotations, atol=1e-12)
    np.testing.assert_allclose(data.qvel, 0.0)
    np.testing.assert_array_equal(data.ctrl, controls)
