"""Referee observation extraction with model and physics data fixtures."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import mujoco
import numpy as np
from bitbots_mujoco_sim.referee import RefereeObservationBuilder

from bitbots_msgs.msg import SimulationState


class RefereeObservationTest(unittest.TestCase):
    def setUp(self):
        self.model = SimpleNamespace(
            ngeom=4,
            geom_contype=[1] * 4,
            geom_conaffinity=[1] * 4,
            geom_aabb=np.array([[0, 0, 0, 0.1, 0.2, 0.3]] * 4),
            njnt=3,
            jnt_type=[mujoco.mjtJoint.mjJNT_FREE] * 3,
            jnt_dofadr=[0, 6, 12],
            qpos0=[1, 2, 3, 0, 0, 0, 0, 4, 5, 6, 0, 0, 0, 0, 7, 8, 9],
            geom_bodyid=[2, 4, 5, 0],
            body_parentid=[0, 0, 1, 0, 3, 0],
            body_jntadr=[-1, 0, -1, 1, -1, 2],
            jnt_qposadr=[0, 7, 14],
            jnt_bodyid=[1, 3, 5],
        )
        self.data = SimpleNamespace(qpos=[1, 2, 3, 0, 0, 0, 0, 4, 5, 6, 0, 0, 0, 0, 7, 8, 9], contact=[], ncon=0)
        self.data.geom_xmat = np.tile(np.eye(3).reshape(9), (4, 1))
        self.data.geom_xpos = np.zeros((4, 3))
        self.data.qvel = [0.0] * 18
        self.data.xmat = [[1, 0, 0, 0, 1, 0, 0, 0, 1]] * 6
        self.builder = RefereeObservationBuilder(self.model, {0: 1, 1: 3}, 2)
        self.stamp = SimulationState().header.stamp

    def test_positions_use_free_joint_coordinates(self):
        message = self.builder.build(self.data, self.stamp, 10)
        self.assertEqual(message.header.frame_id, "world")
        self.assertEqual(message.step_number, 10)
        self.assertEqual(message.ball_position.x, 7.0)
        self.assertEqual([robot.position.x for robot in message.robots], [1.0, 4.0])
        self.data.qpos[0] = 99
        self.assertEqual(message.robots[0].position.x, 1.0)

    def test_contacts_resolve_robot_ancestors_and_both_geom_orders(self):
        self.data.contact = [
            SimpleNamespace(geom1=0, geom2=2, dist=-0.01, efc_address=0),
            SimpleNamespace(geom1=2, geom2=1, dist=0.0, efc_address=1),
            SimpleNamespace(geom1=0, geom2=2, dist=-0.02, efc_address=2),
        ]
        self.data.ncon = len(self.data.contact)

        def contact_force(model, data, index, result):
            result[0] = 3.0

        with patch("mujoco.mj_contactForce", side_effect=contact_force):
            message = self.builder.build(self.data, self.stamp, 1)
        self.assertEqual([robot.robot_index for robot in message.robots if robot.touching_ball], [0, 1])
        self.assertEqual([robot.ball_contact_force for robot in message.robots], [6.0, 3.0])

    def test_proximity_inactive_and_environment_contacts_are_ignored(self):
        self.data.contact = [
            SimpleNamespace(geom1=0, geom2=2, dist=0.01, efc_address=0),
            SimpleNamespace(geom1=1, geom2=2, dist=-0.01, efc_address=-1),
            SimpleNamespace(geom1=3, geom2=2, dist=-0.01, efc_address=1),
        ]
        self.data.ncon = len(self.data.contact)
        message = self.builder.build(self.data, self.stamp, 1)
        self.assertFalse(any(robot.touching_ball for robot in message.robots))

    def test_missing_ball_is_explicit(self):
        builder = RefereeObservationBuilder(self.model, {}, -1)
        message = builder.build(self.data, self.stamp, 1)
        self.assertFalse(message.ball_present)
        self.assertEqual(message.robots, [])

    def test_motion_is_copied_from_root_velocity_and_torso_orientation(self):
        self.data.qvel[0] = 0.4
        self.data.qvel[3] = 0.8
        message = self.builder.build(self.data, self.stamp, 10)
        robot = message.robots[0]
        self.assertTrue(robot.motion_valid)
        self.assertEqual(robot.linear_speed, 0.4)
        self.assertEqual(robot.angular_speed, 0.8)
        self.assertEqual(robot.upright, 1.0)
        self.assertEqual(robot.relative_height, 1.0)

    def test_head_and_body_joint_speeds_are_separate(self):
        model = mujoco.MjModel.from_xml_string("""
            <mujoco><worldbody>
              <body name="robot" pos="0 0 1"><freejoint/><geom size="0.1"/>
                <body><joint name="head_yaw"/><geom size="0.05"/></body>
                <body><joint name="knee"/><geom size="0.05"/></body>
              </body>
            </worldbody></mujoco>
        """)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        head = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "head_yaw")
        knee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "knee")
        data.qvel[model.jnt_dofadr[head]] = 0.8
        data.qvel[model.jnt_dofadr[knee]] = -0.4
        builder = RefereeObservationBuilder(model, {0: 1}, -1)
        robot = builder.build(data, self.stamp, 1).robots[0]
        self.assertEqual(robot.head_joint_speed, 0.8)
        self.assertEqual(robot.body_joint_speed, 0.4)

    def test_bounds_include_rotated_collision_geometry(self):
        self.data.geom_xpos[0] = [1, 2, 3]
        self.data.geom_xmat[0] = [0, -1, 0, 1, 0, 0, 0, 0, 1]
        robot = self.builder.build(self.data, self.stamp, 1).robots[0]
        self.assertTrue(robot.bounds_valid)
        self.assertAlmostEqual(robot.min_x, 0.8)
        self.assertAlmostEqual(robot.max_x, 1.2)
        self.assertAlmostEqual(robot.min_y, 1.9)
        self.assertAlmostEqual(robot.max_y, 2.1)

    def test_ball_blockage_uses_actual_surrounding_geometry(self):
        model = mujoco.MjModel.from_xml_string("""
            <mujoco><worldbody>
              <body name="robot" pos="0 0 1"><freejoint/><geom size="0.05"/>
                <geom type="box" pos="0.2 0 -0.9" size="0.01 0.2 0.1"/>
                <geom type="box" pos="-0.2 0 -0.9" size="0.01 0.2 0.1"/>
                <geom type="box" pos="0 0.2 -0.9" size="0.2 0.01 0.1"/>
                <geom type="box" pos="0 -0.2 -0.9" size="0.2 0.01 0.1"/>
              </body>
              <body name="ball" pos="0 0 0.1"><freejoint/><geom size="0.07"/></body>
            </worldbody></mujoco>
        """)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        builder = RefereeObservationBuilder(model, {0: 1}, 1)
        robot = builder.build(data, self.stamp, 1).robots[0]
        self.assertEqual(robot.ball_blockage, 1.0)
        self.assertTrue(robot.heading_valid)
        self.assertAlmostEqual(robot.yaw, 0.0)
        data.xmat[1][8] = 0.0
        robot = builder.build(data, self.stamp, 2).robots[0]
        self.assertEqual(robot.ball_blockage, 0.0)

    def test_robot_contact_forces_aggregate_independent_of_geom_order(self):
        self.data.contact = [
            SimpleNamespace(
                geom1=0, geom2=1, dist=-0.01, efc_address=0, pos=np.zeros(3), frame=[1, 0, 0, 0, 1, 0, 0, 0, 1]
            ),
            SimpleNamespace(
                geom1=1, geom2=0, dist=-0.01, efc_address=1, pos=np.zeros(3), frame=[-1, 0, 0, 0, 1, 0, 0, 0, -1]
            ),
        ]
        self.data.ncon = len(self.data.contact)

        def force(model, data, index, output):
            output[0] = 10.0

        def velocity(model, data, kind, geom, output, local):
            output[3] = 0.1 if geom == 0 else 0.0

        with (
            patch("bitbots_mujoco_sim.referee.mujoco.mj_contactForce", side_effect=force),
            patch("bitbots_mujoco_sim.referee.mujoco.mj_objectVelocity", side_effect=velocity),
        ):
            message = self.builder.build(self.data, self.stamp, 1)
        self.assertEqual(len(message.robot_contacts), 1)
        contact = message.robot_contacts[0]
        self.assertEqual((contact.robot_a, contact.robot_b), (0, 1))
        self.assertEqual(contact.force, 20.0)
        self.assertAlmostEqual(contact.approach_a, 0.1)
        self.assertEqual(contact.approach_b, 0.0)
