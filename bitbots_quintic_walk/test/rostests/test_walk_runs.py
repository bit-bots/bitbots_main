#!/usr/bin/env python3
from bitbots_test.test_case import RosNodeTestCase
from bitbots_test.mocks import MockSubscriber
from geometry_msgs.msg import Twist
from bitbots_msgs.msg import JointCommand
import rospy


class WalkRunsTestCase(RosNodeTestCase):
    def test_walk_runs(self):
        sub = MockSubscriber('walking_motor_goals', JointCommand)
        self.with_assertion_grace_period(lambda: self.assertNodeRunning("walking"), 10)
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1, latch=True)

        goal = Twist()
        goal.linear.x = 0.3
        pub.publish(goal)
        sub.wait_until_connected()
        rospy.sleep(5)
        self.with_assertion_grace_period(sub.assertMessageReceived, 100)

if __name__ == '__main__':
    from bitbots_test import run_rostests
    run_rostests(WalkRunsTestCase)
