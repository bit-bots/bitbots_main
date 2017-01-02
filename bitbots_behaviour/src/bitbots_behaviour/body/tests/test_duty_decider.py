# -*- coding:utf-8 -*-
"""
TestDutyDecider
^^^^^^^^^^^^^^^

.. moduleauthor:: sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 4/18/14: Created (sheepy)

"""
import unittest

import mock
import bitbots.modules.behaviour.body.decisions.common.duty_decider
from bitbots.modules.behaviour.body.decisions.common.duty_decider import DutyDecider
from bitbots.modules.behaviour.body.decisions.goalie.goalie_decision import GoalieDecision
from bitbots.modules.behaviour.body.decisions.kick_off.kick_off import KickOff
from bitbots.modules.behaviour.body.decisions.common.role_decider import RoleDecider
from bitbots.modules.behaviour.modell.connector import Connector
from bitbots.modules.keys import DATA_KEY_GAME_STATUS, DATA_VALUE_STATE_PLAYING


class TestDutyDecider(unittest.TestCase):
    def perform_duty_decider_call_with_start_duty(self, start_duty=None):
        push_mock = mock.MagicMock()

        dc = DutyDecider(None)
        dc.push = push_mock

        data = {DATA_KEY_GAME_STATUS: DATA_VALUE_STATE_PLAYING}
        connector = Connector(data)

        bitbots.modules.behaviour.body.decisions.common.duty_decider.duty = start_duty

        dc.perform(connector)
        return push_mock, connector

    def test_goes_to_fieldie_on_no_argument(self):
        push_mock, connector = self.perform_duty_decider_call_with_start_duty()

        self.assertEquals("TeamPlayer", connector.get_duty())
        self.assertEquals(1, push_mock.call_count)
        self.assertEquals(KickOff, push_mock.call_args[0][0])

    def test_goes_to_goalie_on_argument(self):
        push_mock, connector = self.perform_duty_decider_call_with_start_duty("Goalie")

        self.assertEquals("Goalie", connector.get_duty())
        self.assertEquals(1, push_mock.call_count)
        self.assertEquals(GoalieDecision, push_mock.call_args[0][0])


if __name__ == '__main__':
    unittest.main()
