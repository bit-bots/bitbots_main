#!/usr/bin/env python3
from bitbots_test.test_case import RosNodeTestCase
from bitbots_test.mocks import MockSubscriber
from actionlib_msgs.msg import GoalStatus
from bitbots_msgs.msg import DynUpAction, DynUpGoal, JointCommand
import actionlib


class DynupRunsTestCase(RosNodeTestCase):
    def one_run(self, direction):
        def done_cb(state, result):
            assert state == GoalStatus.SUCCEEDED, "Dynup was not successful"
            self.kick_succeeded = True

        self.kick_succeeded = False

        sub = MockSubscriber('dynup_motor_goals', JointCommand)
        self.with_assertion_grace_period(lambda: self.assertNodeRunning("dynup"), 20)
        client = actionlib.SimpleActionClient('dynup', DynUpAction)
        assert client.wait_for_server(), "Dynup action server not running"

        goal = DynUpGoal()
        goal.direction = direction
        client.send_goal(goal)
        client.done_cb = done_cb
        client.wait_for_result()
        sub.wait_until_connected()
        sub.assertMessageReceived()
        assert self.kick_succeeded, "Dynup was not successful"

    def test_walkready(self):
        self.one_run("walkready")

    def test_front(self):
        self.one_run("front")

    def test_back(self):
        self.one_run("back")

    def test_rise(self):
        self.one_run("rise")

    def test_descend(self):
        self.one_run("descend")

if __name__ == '__main__':
    from bitbots_test import run_rostests
    run_rostests(DynupRunsTestCase)
