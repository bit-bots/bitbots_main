"""Base classes for TestCases as well as useful assertions and ros integrations"""
from unittest.case import TestCase
import rospy


class RosNodeTestCase(TestCase):
    """
    A TestCase class specialized for running tests as a ROS-Node

    In detail it does the following:

    - set up and teardown a ros node automatically
    """

    def setUp(self) -> None:
        super().setUp()
        rospy.init_node(type(self).__name__, anonymous=True)

    def tearDown(self) -> None:
        super().tearDown()

    @property
    def topic(self):
        """A ros topic which is local to the current test"""
        if self._testMethodName:
            return f"{type(self).__name__}/{self._testMethodName}"
        return f"{type(self).__name__}"
