"""Base classes for TestCases as well as useful assertions and ros integrations"""
from typing import *
import time
from datetime import datetime, timedelta
from unittest.case import TestCase as BaseTestCase
import rospy


class GeneralAssertionMixins:
    """Supplementary ROS independent assertions"""

    def with_assertion_grace_period(self, f: Callable, t: int = 500, *args, **kwargs):
        """
        Execute the provided callable repeatedly while suppressing AssertionErrors until the grace period
        has exceeded.

        :param f: The function which gets repeatedly called. Any raised AssertionErrors are caught.
            All additional parameters from *args and **kwargs are passed to this function.
        :param t: Grace period time in milliseconds
        """
        start = datetime.now()
        while True:
            try:
                return f(*args, **kwargs)
            except AssertionError:
                if start + timedelta(milliseconds=t) < datetime.now():
                    raise

                time.sleep(min(10, int(t / 10)) / 1000)


class TestCase(GeneralAssertionMixins, BaseTestCase):
    """A Bit-Bots specific TestCase class from which all other TestCases should inherit"""
    pass


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
