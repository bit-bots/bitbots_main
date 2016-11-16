import unittest
import math
from bitbots.util import get_config
from bitbots.robot.kinematics import Robot
from bitbots.robot.pypose import PyPose
from bitbots.util.pydatavector import PyDataVector
from bitbots.util.kinematicutil import get_robot_horizon_p_deg
from bitbots.util.kinematicutil import get_robot_horizon_r
from bitbots.util.kinematicutil import rtd, dtr

import numpy as np

__author__ = 'robert'

class TestKinematicUtil(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        print "#### Test Kinematic Utils ####"

    def init_robot_and_pose(self, roll_angle=0, pitch_angle=0):
        pose = PyPose()
        config = get_config()
        config["RobotTypeName"] = "Hambot"
        internal_camera_angle = config[config["RobotTypeName"]]["Head"][3]["rpy"][1]
        robot = Robot()
        pose.belly_pitch.position = -pitch_angle
        pose.belly_roll.position = -roll_angle
        assert pose.belly_roll.position == -roll_angle
        assert pose.belly_pitch.position ==- pitch_angle
        robot.update(pose)
        return (robot, pose, internal_camera_angle)

    def test_get_robot_tilt(self):
        robot, pose , internal_camera_angle= self.init_robot_and_pose()
        cam_angles = get_robot_horizon_p_deg(robot, 1)
        self.assertEquals(internal_camera_angle, -13)
        self.assertTrue(abs(cam_angles[0] + internal_camera_angle) < 1e-10)

    def test_get_robot_tilt2(self):
        angle = 13
        robot, pose , internal_camera_angle= self.init_robot_and_pose(pitch_angle=angle)
        cam_angles = get_robot_horizon_p_deg(robot, 1)
        self.assertEquals(internal_camera_angle, -13)
        self.assertTrue(abs(cam_angles[0] + internal_camera_angle - angle) < 1e-10)

    def test_get_robot_pan_default(self):
        robot, pose, _ = self.init_robot_and_pose()
        cam_angles = get_robot_horizon_p_deg(robot, 1)
        self.assertTrue(abs(cam_angles[1]) < 1e-10)

    def test_get_robot_pan(self):
        angle = 10
        robot, pose, _ = self.init_robot_and_pose(roll_angle=angle)
        cam_angles = get_robot_horizon_p_deg(robot, 1)
        self.assertTrue(abs(cam_angles[1] - angle) < 1e-10)

    def test_get_robot_pan2(self):
        angle = -10
        robot, pose, _ = self.init_robot_and_pose(roll_angle=angle)
        cam_angles = get_robot_horizon_p_deg(robot, 1)
        self.assertTrue(abs(cam_angles[1] - angle) < 1e-10)

    def test_get_robot_pitch_rpy(self):
        robot, pose, internal_angle = self.init_robot_and_pose()
        self.assertEquals(internal_angle, -13)
        cam_angles = get_robot_horizon_r(robot, PyDataVector(0,internal_angle,0)) * rtd()
        self.assertTrue(abs(cam_angles[1]) < 1e-10)
        self.assertTrue(abs(cam_angles[0]) < 1e-10)

    def test_get_robot_pitch_rpy2(self):
        robot, pose, internal_angle = self.init_robot_and_pose()
        #robot.set_initial_angles(0, internal_angle, 0)
        self.assertEquals(internal_angle, -13)
        cam_angles = get_robot_horizon_r(robot, PyDataVector(0,-internal_angle,0)) * rtd()
        self.assertTrue(abs(cam_angles[1]) < 1e-10)
        self.assertTrue(abs(cam_angles[0] + 2 * internal_angle) < 1e-10)

    def test_get_robot_roll_rpy(self):
        robot, pose, internal_angle = self.init_robot_and_pose()
        cam_angles = get_robot_horizon_r(robot, PyDataVector(internal_angle,0,0)) * rtd()
        self.assertEquals(internal_angle, -13)
        self.assertTrue(abs(cam_angles[1] - internal_angle) < 1e-10)

    def test_get_robot_roll_rpy2(self):
        robot, pose, internal_angle = self.init_robot_and_pose()
        cam_angles = get_robot_horizon_r(robot, PyDataVector(-internal_angle,0,0)) * rtd()
        self.assertEquals(internal_angle, -13)
        self.assertTrue(abs(cam_angles[1] + internal_angle) < 1e-10)

if __name__ == '__main__':
    unittest.main()
