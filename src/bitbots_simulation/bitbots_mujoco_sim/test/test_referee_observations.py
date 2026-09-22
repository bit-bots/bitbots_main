"""Referee observation extraction with model and physics data fixtures."""

import unittest
from types import SimpleNamespace

from bitbots_msgs.msg import SimulationState
from bitbots_mujoco_sim.referee import RefereeObservationBuilder


class RefereeObservationTest(unittest.TestCase):
    def setUp(self):
        self.model = SimpleNamespace(
            ngeom=4,
            geom_bodyid=[2, 4, 5, 0],
            body_parentid=[0, 0, 1, 0, 3, 0],
            body_jntadr=[-1, 0, -1, 1, -1, 2],
            jnt_qposadr=[0, 7, 14],
            jnt_bodyid=[1, 3, 5],
        )
        self.data = SimpleNamespace(qpos=[1, 2, 3, 0, 0, 0, 0, 4, 5, 6, 0, 0, 0, 0, 7, 8, 9], contact=[], ncon=0)
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
        message = self.builder.build(self.data, self.stamp, 1)
        self.assertEqual([robot.robot_index for robot in message.robots if robot.touching_ball], [0, 1])

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
