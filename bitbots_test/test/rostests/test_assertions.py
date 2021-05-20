#!/usr/bin/env python3
import rospy
from rosgraph_msgs.msg import Log as LogMsg
from bitbots_test.test_case import RosNodeTestCase


class RosoutAssertionMixinTestCase(RosNodeTestCase):
    def test_assert_ros_logs(self):
        with self.subTest("nothing was published"):
            self.assertRaises(AssertionError, self.assertRosLogs)

        rospy.loginfo("This is a test message")

        # no parameters
        self.with_assertion_grace_period(self.assertRosLogs, t=1000)

        # with message parameter
        self.assertRosLogs(msg=r'is a test')
        self.assertRaises(AssertionError, lambda: self.assertRosLogs(msg=r'non-logged message'))

        # with log level specified
        self.assertRosLogs(level=[LogMsg.INFO])
        self.assertRaises(AssertionError, lambda: self.assertRosLogs(level=[LogMsg.DEBUG, LogMsg.WARN, LogMsg.ERROR, LogMsg.FATAL]))

        # with node specified
        self.assertRosLogs(node=rospy.get_name())
        self.assertRaises(AssertionError, lambda: self.assertRosLogs(node="/non_existing"))

    def test_assert_not_ros_logs(self):
        with self.subTest("nothing was published"):
            self.assertNotRosLogs()

        rospy.loginfo("This is a test message")
        self.with_assertion_grace_period(self.assertRosLogs, t=1000)

        # no parameters
        self.assertRaises(AssertionError, lambda: self.assertNotRosLogs())

        # with message parameter
        self.assertRaises(AssertionError, lambda: self.assertNotRosLogs(msg="is a test"))
        self.assertNotRosLogs(msg="non-logged message")

        # with log level specified
        self.assertRaises(AssertionError, lambda: self.assertNotRosLogs(level=[LogMsg.INFO]))
        self.assertNotRosLogs(level=[LogMsg.DEBUG, LogMsg.WARN, LogMsg.ERROR, LogMsg.FATAL])

        # with node specified
        self.assertRaises(AssertionError, lambda: self.assertNotRosLogs(node=rospy.get_name()))
        self.assertNotRosLogs(node="/non_existing")


if __name__ == "__main__":
    from bitbots_test import run_rostests

    run_rostests(RosoutAssertionMixinTestCase)
