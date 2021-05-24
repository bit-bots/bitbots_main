"""Base classes for TestCases as well as useful assertions and ros integrations"""
import socket
from typing import *
import time
from datetime import datetime, timedelta
from unittest.case import TestCase as BaseTestCase
from xmlrpc.client import ServerProxy

import rospy
import re
from rosgraph_msgs.msg import Log as LogMsg
import rosgraph


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

    def assertInRange(self, value: Union[int, float], range_limit: range):
        """
        Assert that a value is inside a given range.

        :param value: The value which is verified to be in the range.
        :param range_limit: Enclosing range in which *value* is verified to be inside.
        """
        # note: python ranges start values are inclusive and stop values are exclusive
        if range_limit.step > 0:
            # ranges with positive steps
            if value < range_limit.start or value >= range_limit.stop:
                raise AssertionError(f"value {value} is not in range {range_limit}")
        else:
            # ranges with negative steps
            if value > range_limit.start or value <= range_limit.stop:
                raise AssertionError(f"value {value} is not in range {range_limit}")

    def assertNotInRange(self, value: Union[int, float], range_limit: range):
        """
        Assert that a value is not inside a given range.

        :param value: The value which is verified to not be in the range.
        :param range_limit: A range object in which *value* is verified to **not** be inside
        """
        with self.assertRaises(AssertionError, msg=f"value {value} is in range {range_limit}"):
            self.assertInRange(value, range_limit)


class RosLogAssertionMixins:
    """Supplementary assertions for roslog """
    rosout_subscriber: rospy.Subscriber
    rosout_buffer: List[LogMsg]
    rosout_max_size: int

    def setup_roslog_assertions(self, aggregator_max_size: int = 4096):
        """
        Setup :class:`RosLogAssertionMixins`s internal subscriber.

        This is necessary for logging assertions to work because the internal subscriber puts rosout messages
        in a buffer which all rosout assertions use.

        :param aggregator_max_size: How many messages form rosout to keep in the internal buffer
        """
        self.rosout_max_size = aggregator_max_size
        self.rosout_buffer = []
        # http://wiki.ros.org/rosout
        self.rosout_subscriber = rospy.Subscriber("/rosout", LogMsg, callback=self._rosout_callback, queue_size=10)

    def teardown_roslog_assertions(self):
        self.rosout_subscriber.unregister()
        self.rosout_buffer.clear()

    def _rosout_callback(self, msg: LogMsg):
        self.rosout_buffer.append(msg)
        while len(self.rosout_buffer) > self.rosout_max_size:
            self.rosout_buffer.pop(0)

    def assertRosLogs(self, node: str = r'.*', msg: str = r'.*', level: List[int] = None):
        """
        Assert that a node has logged a message with a certain text on a specific log-level.

        If any parameter is not specified, every message fulfills that requirement.
        This means that when called with no arguments, any roslog entry logged by any node on any
        loglevel makes this assertion pass.

        :param node: A regex which is matched against a roslog entries originating node
        :param msg: A regex which is matched against a roslog entries content
        :param level: A list of log levels which are considered valid during assertion
        """
        if level is None:
            level = [LogMsg.DEBUG, LogMsg.INFO, LogMsg.WARN, LogMsg.ERROR, LogMsg.FATAL]

        for i_msg in self.rosout_buffer:
            if re.search(node, i_msg.name) is not None and re.search(msg,
                                                                     i_msg.msg) is not None and i_msg.level in level:
                return

        raise AssertionError(f"No such message was published to rosout (node={node}, msg={msg}, level in {level})")

    def assertNotRosLogs(self, node: str = r'.*', msg: str = r'.*', level: List[int] = None):
        """
        Assert that no node matching the `node` regex logged a message matching the `msg` regex on one of
        the log levels from `level`.

        If any parameter is not specified, every message fulfills that requirement.
        This means that when called with no arguments, any roslog entry logged by any node on any
        loglevel makes this assertion fail.

        :param node: A regex which is matched against a roslog entries originating node
        :param msg: A regex which is matched against a roslog entries content
        :param level: A list of log levels to which checking should be restricted
        :return:
        """
        if level is None:
            level = [LogMsg.DEBUG, LogMsg.INFO, LogMsg.WARN, LogMsg.ERROR, LogMsg.FATAL]

        for i_msg in self.rosout_buffer:
            if re.search(node, i_msg.name) is not None and re.search(msg,
                                                                     i_msg.msg) is not None and i_msg.level in level:
                raise AssertionError(f"Roslog entry {i_msg} was logged")


class RosNodeAssertionMixins:
    """Supplementary assertions for validating the state of other ROS nodes"""
    ros_master: rosgraph.Master

    @classmethod
    def setup_ros_node_assertions(cls):
        cls.ros_master = rosgraph.Master(rospy.get_name())

    @classmethod
    def assertNodeRunning(cls, node_name: str):
        """
        Assert that a node with name `node_name` is running and responding to pings

        :arg node_name: The name of the node which should be running
        """
        try:
            node_api = cls.ros_master.lookupNode(node_name)
            node = ServerProxy(node_api)
            node.getPid(rospy.get_name())  # this is the ping
        except (rosgraph.MasterError, ConnectionError) as e:
            raise AssertionError(f"ros node {node_name} is not running") from e


class TestCase(GeneralAssertionMixins, BaseTestCase):
    """A Bit-Bots specific TestCase class from which all other TestCases should inherit"""
    pass


class RosNodeTestCase(RosLogAssertionMixins, RosNodeAssertionMixins, TestCase):
    """
    A TestCase class specialized for running tests as a ROS-Node

    In detail it does the following:

    - set up and teardown a ros node automatically
    - set up roslog assertions
    - set up ros node assertions
    """

    def setUp(self) -> None:
        super().setUp()
        rospy.init_node(type(self).__name__, anonymous=True)
        self.setup_roslog_assertions()

    def tearDown(self) -> None:
        super().tearDown()
        self.teardown_roslog_assertions()

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        cls.setup_ros_node_assertions()

    @property
    def topic(self):
        """A ros topic which is local to the current test"""
        if self._testMethodName:
            return f"{type(self).__name__}/{self._testMethodName}"
        return f"{type(self).__name__}"


class WebotsTestCase(RosNodeTestCase):
    """A specific TestCase class which offers utility methods when running in a webots simulator"""
    SUPERVISOR_NODE_NAME = "/webots_ros_supervisor"

    ros_master: rosgraph.Master

    def reset_simulation(self):
        pass

    @classmethod
    def wait_for_simulator(cls, timeout: int = 120):
        """
        Wait until the webots supervisor node is running because this implies that webots is running

        :param timeout: Timout in seconds after which this method aborts
        """
        start_time = time.time()
        while True:
            try:
                cls.assertNodeRunning(cls.SUPERVISOR_NODE_NAME)
                break
            except AssertionError:
                if start_time + timeout < time.time():
                    raise
                time.sleep(0.01)

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()
        cls.wait_for_simulator()

    def setUp(self) -> None:
        super().setUp()
        self.reset_simulation()
