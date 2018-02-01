# -*- coding:utf-8 -*-
"""
TestConfirm
^^^^^^^^^^^

.. moduleauthor:: sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 5/7/14: Created (sheepy)

"""
from collections import namedtuple
import unittest
import mock
import time
from bitbots.modules.behaviour.head.actions.track_object import TrackBall
from bitbots.modules.behaviour.modell.connector import Connector
from bitbots.modules.keys import DATA_KEY_BALL_FOUND, DATA_KEY_BALL_INFO
from bitbots.robot.pypose import PyPose
from bitbots.util import get_config


class TestConfirm(unittest.TestCase):
    def test_based_on_duty_y_center_for_tracking_is_set_correctly(self):
        behaviour_mock = mock.MagicMock()

        confirm = TrackBall()
        confirm.setup_internals(behaviour_mock, {})

        nt = namedtuple("x", "y")
        nt.x = 0
        nt.y = 0
        connector = Connector(data={
            DATA_KEY_BALL_FOUND: True,
            DATA_KEY_BALL_INFO: nt,
            "BallLastSeen": rospy.get_time(),
            "Pose": PyPose(),
            "Ipc": mock.MagicMock()
        })
        connector.set_duty("Goalie")

        self.assertEquals("Goalie", connector.get_duty())

        confirm.perform(connector)
        confirm.perform(connector)
        confirm.perform(connector)
        confirm.perform(connector)

        self.assertEqual(get_config()["Behaviour"]["Common"]["Tracking"]["yCenterGoalie"], confirm.b_center_goalie)
