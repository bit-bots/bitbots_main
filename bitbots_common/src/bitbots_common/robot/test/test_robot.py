import unittest;import math;import numpy as np;from bitbots.util import get_config;from bitbots.robot.kinematics import Robot, KinematicTask;from bitbots.robot.pypose import PyPose

__author__ = 'robert'

class TestKinematics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        print "#### Test Robot Python Interface ####"

    def test_robot_is_created_from_config(self):
        try:
            robot=Robot()
            larm = robot.get_l_arm_chain()
            self.assertTrue(larm is not None)
        except Exception, e:
            self.assertEquals(e, None)

    def test_interface_is_callable(self):
        print "### interface ###"
        robot = Robot()

        robot.get_joint_by_id(0)
        robot.get_joint_by_name("Root")
        #robot.debugprint()
        robot.update(PyPose())
        robot.inverse_chain_t(3, (0,0,-300), 1, 100, 3)
        robot.inverse_chain(3, np.array((0,0,-300)), 1, 100, 3)
        robot.get_centre_of_gravity()
        robot.set_angles_to_pose(PyPose(), 0, -1)

    def test_GOAL_config(self):
        print "### Goal ###"
        config = get_config()
        config["RobotTypeName"] = "Hambot"
        robot = Robot()
        larm = robot.get_l_arm_chain()

    def test_Darwin_config(self):
        print "### Darwin ###"
        config = get_config()
        config["RobotTypeName"] = "Darwin"
        robot = Robot()
        larm = robot.get_l_arm_chain()

    def test_chain_masspoints(self):
        print "### Masspoint ###"
        config = get_config()
        config["RobotTypeName"] = "Darwin"
        robot = Robot()
        task = KinematicTask(robot)
        r_chain = task.create_cog_chain(3)
        l_chain = task.create_cog_chain(4)
        robot.update(PyPose())
        task.update_chain(l_chain, 3)
        l_cmp = l_chain.get_joint(6).get_chain_masspoint()
        task.update_chain(r_chain, 3)
        r_cmp = r_chain.get_joint(6).get_chain_masspoint()
        r_cmp[1] = -r_cmp[1]
        diff = l_cmp - r_cmp
        diff = np.where(diff < 0, -diff, diff)
        max = diff.max()
        if(not diff.max() < 1e-5):
            print "Chain masspoints differ:\n%s\n%s\t%s" % (diff, l_cmp,r_cmp)

        print "\n FALSE DIFF\n %s \n" % diff
        #self.assertTrue(diff.max() < 1e-5)

    def test_maspoint_simple_l(self):
        print "### Masspoint simple left ###"
        config = get_config()
        config["RobotTypeName"] = "Darwin"
        robot = Robot()
        task = KinematicTask(robot)
        r_chain = task.create_cog_chain(3)
        l_chain = task.create_cog_chain(4)
        robot.update(PyPose())

        cog=robot.get_centre_of_gravity()
        lfi = robot.get_joint_by_id(35).get_chain_matrix(inverse=True)
        rfi = robot.get_joint_by_id(34).get_chain_matrix(inverse=True)

        rcog = np.dot(rfi, cog)
        lcog = np.dot(lfi, cog)

        task.update_chain(l_chain, 3)
        l_cmp = l_chain.get_joint(6).get_chain_masspoint()

        diff = l_cmp - lcog[0:3]

        print diff

    def test_maspoint_simple_r(self):
        print "### Masspoint simple right ###"
        config = get_config()
        config["RobotTypeName"] = "Darwin"
        robot = Robot()
        task = KinematicTask(robot)
        r_chain = task.create_cog_chain(3)
        l_chain = task.create_cog_chain(4)
        robot.update(PyPose())

        cog=robot.get_centre_of_gravity()
        lfi = robot.get_joint_by_id(35).get_chain_matrix(inverse=True)
        rfi = robot.get_joint_by_id(34).get_chain_matrix(inverse=True)

        rcog = np.dot(rfi, cog)
        lcog = np.dot(lfi, cog)

        task.update_chain(r_chain, 3)
        r_cmp = r_chain.get_joint(6).get_chain_masspoint()

        diff = r_cmp - rcog[0:3]

        print diff


if "main" in __name__:
    unittest.main()
