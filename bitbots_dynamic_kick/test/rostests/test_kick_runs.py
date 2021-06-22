#!/usr/bin/env python3
from bitbots_test.test_case import RosNodeTestCase
from bitbots_test.mocks import MockSubscriber
from actionlib_msgs.msg import GoalStatus
from bitbots_msgs.msg import KickAction, KickGoal, JointCommand
from geometry_msgs.msg import Quaternion
import actionlib
import rospy


class KickRunsTestCase(RosNodeTestCase):
    def test_kick_runs(self):
        def done_cb(state, result):
            assert state == GoalStatus.SUCCEEDED, "Kick was not successful"
            self.kick_succeeded = True

        self.kick_succeeded = False

        sub = MockSubscriber('kick_motor_goals', JointCommand)
        self.with_assertion_grace_period(lambda: self.assertNodeRunning("dynamic_kick"), 20)
        client = actionlib.SimpleActionClient('dynamic_kick', KickAction)
        assert client.wait_for_server(), "Kick action server not running"

        goal = KickGoal()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "base_footprint"
        goal.ball_position.x = 0.2
        goal.kick_direction = Quaternion(0, 0, 0, 1)
        goal.kick_speed = 1
        client.send_goal(goal)
        client.done_cb = done_cb
        client.wait_for_result()
        sub.wait_until_connected()
        sub.assertMessageReceived()
        assert self.kick_succeeded, "Kick was not successful"

if __name__ == '__main__':
    from bitbots_test import run_rostests
    run_rostests(KickRunsTestCase)
