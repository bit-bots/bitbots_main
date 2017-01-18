# -*- coding:utf-8 -*-
"""

History:
    * 2015-09-30 Timon: Test umgeschrieben so dass die limits dynamisch erkannt werden.
      Ich denke das was hier getestet wird gehört eigentlich in einen IPC-Test, da
      die Exceptions die hier geprüft werden dort geworfen werden.
"""
import unittest
from bitbots.modules.behaviour.head.actions.head_to_pan_tilt import HeadToPanTilt
from bitbots.modules.behaviour.modell.connector import Connector
from bitbots.modules.keys import DATA_KEY_BALL_FOUND
from bitbots.ipc import SharedMemoryIPC
from bitbots.util import Joints


class TestHeadPanTilt(unittest.TestCase):

    def setup_connector(self):
        smi = SharedMemoryIPC()
        connector = Connector(data={
            DATA_KEY_BALL_FOUND: False,
            "Pose": smi.get_pose(),
            "Ipc": smi
        })
        connector.set_duty("Goalie")
        return connector

    def had_pan_tilt_limit_expect_exception(self, pan, tilt):
        connector = self.setup_connector()
        hpt = HeadToPanTilt((pan, tilt))

        with self.assertRaises(ValueError):
            hpt.perform(connector)

    def had_pan_tilt_limit_expect_no_exception(self, pan, tilt):
        connector = self.setup_connector()
        hpt = HeadToPanTilt((pan, tilt))

        hpt.perform(connector)

    def test_head_pan_tilt_limits(self):
        # This assures that the motor limits for the head throwing exceptions
        joints = Joints()
        pan = joints.get_joint_by_name("HeadPan")
        tilt = joints.get_joint_by_name("HeadTilt")
        self.had_pan_tilt_limit_expect_exception(pan.limit_min - 1, 0)
        self.had_pan_tilt_limit_expect_exception(pan.limit_max + 1, 0)
        self.had_pan_tilt_limit_expect_exception(0, tilt.limit_min - 1)
        self.had_pan_tilt_limit_expect_exception(0, tilt.limit_max + 1)

        # This assures the motor limits that are inside the bounds do not raise
        # an exception
        self.had_pan_tilt_limit_expect_no_exception(pan.limit_min, 0)
        self.had_pan_tilt_limit_expect_no_exception(pan.limit_max, 0)
        self.had_pan_tilt_limit_expect_no_exception(0, tilt.limit_min)
        self.had_pan_tilt_limit_expect_no_exception(0, tilt.limit_max)
