#!/usr/bin/env python3
from bitbots_test.test_case import RosNodeTestCase
from dynamic_stack_decider import DSD
from bitbots_blackboard.blackboard import BodyBlackboard
from pathlib import Path
import rospy
import rospkg

class DsdFileTestCase(RosNodeTestCase):
    def test_dsd_valid(self):
        D = DSD(BodyBlackboard())
        behavior_dir = Path(rospkg.RosPack().get_path('bitbots_body_behavior')) / 'src' / 'bitbots_body_behavior'
        D.register_actions(behavior_dir / 'actions')
        D.register_decisions(behavior_dir / 'decisions')
        D.load_behavior(behavior_dir / 'main.dsd')


if __name__ == '__main__':
    from bitbots_test import run_rostests
    run_rostests(DsdFileTestCase)
