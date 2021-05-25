"""Base classes for TestCases as well as useful assertions and ros integrations"""
from typing import *
import time
import rospy
import re
import rosgraph
import geometry_msgs.msg
from rosgraph_msgs.msg import Log as LogMsg
import std_srvs.srv
import bitbots_msgs.srv
import gazebo_msgs.msg
from datetime import datetime, timedelta
from unittest.case import TestCase as BaseTestCase
from xmlrpc.client import ServerProxy


class GeneralAssertionMixins:
    """Supplementary ROS independent assertions"""
    def with_assertion_grace_period(self, f: Callable, t: int = 500, *args, **kwargs):
        """
        Execute the provided callable repeatedly while suppressing AssertionErrors until the grace period
        has exceeded.

        :param f: The function which gets repeatedly called. Any raised AssertionErrors are caught.
            All additional parameters from *args* and *kwargs* are passed to this function.
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

    def assertInRange(self, value: Union[int, float], range: Tuple[Union[int, float], Union[int, float]]):
        """
        Assert that a value is inside a given range.

        :param value: The value which is verified to be in the range.
        :param range: Range in which *value* should be. Specified as a two element tuple that contains the two inclusive
            ends of the range.
        """
        min_value = min(range[0], range[1])
        max_value = max(range[0], range[1])

        if value < min_value or value > max_value:
            raise AssertionError(f"value {value} is not in range {range}")

    def assertNotInRange(self, value: Union[int, float], range: Tuple[Union[int, float], Union[int, float]]):
        """
        Assert that a value is not inside a given range.

        :param value: The value which is verified to not be in the range.
        :param range: Range in which *value* should **not** be. Specified as a two element tuple that contains the two
            inclusive ends of the range.
        """
        with self.assertRaises(AssertionError, msg=f"value {value} is in range {range}"):
            self.assertInRange(value, range)


class TestCase(GeneralAssertionMixins, BaseTestCase):
    """A Bit-Bots specific TestCase class from which all other TestCases should inherit"""
    pass


class RosLogAssertionMixins:
    """Supplementary assertions for roslog """
    _sub_rosout: rospy.Subscriber
    _rosout_buffer: List[LogMsg]
    _rosout_buffer_max_size: int

    def setup_roslog_assertions(self, aggregator_max_size: int = 4096):
        """
        Setup internal subscriber and buffer.

        This is necessary for logging assertions to work because the internal subscriber puts rosout messages
        in a buffer which all rosout assertions use.

        :param aggregator_max_size: How many messages form rosout to keep in the internal buffer
        """
        self._rosout_buffer_max_size = aggregator_max_size
        self._rosout_buffer = []
        # http://wiki.ros.org/rosout
        self._sub_rosout = rospy.Subscriber("/rosout", LogMsg, callback=self._rosout_callback, queue_size=10)

    def teardown_roslog_assertions(self):
        self._sub_rosout.unregister()
        self._rosout_buffer.clear()

    def _rosout_callback(self, msg: LogMsg):
        self._rosout_buffer.append(msg)
        while len(self._rosout_buffer) > self._rosout_buffer_max_size:
            self._rosout_buffer.pop(0)

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

        for i_msg in self._rosout_buffer:
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

        for i_msg in self._rosout_buffer:
            if re.search(node, i_msg.name) is not None and re.search(msg,
                                                                     i_msg.msg) is not None and i_msg.level in level:
                raise AssertionError(f"Roslog entry {i_msg} was logged")


class RosNodeAssertionMixins:
    """Supplementary assertions for validating the state of other ROS nodes"""
    _ros_master: rosgraph.Master

    @classmethod
    def setup_ros_node_assertions(cls):
        cls._ros_master = rosgraph.Master(rospy.get_name())

    @classmethod
    def assertNodeRunning(cls, node_name: str):
        """
        Assert that a node with name `node_name` is running and responding to pings

        :arg node_name: The name of the node which should be running
        """
        try:
            node_api = cls._ros_master.lookupNode(node_name)
            node = ServerProxy(node_api)
            node.getPid(rospy.get_name())  # this is the ping
        except (rosgraph.MasterError, ConnectionError) as e:
            raise AssertionError(f"ros node {node_name} is not running") from e


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
    _SUPERVISOR_NODE_NAME = "/webots_ros_supervisor"

    _ros_master: rosgraph.Master
    _latest_model_states: Optional[gazebo_msgs.msg.ModelStates] = None
    _latest_model_states_time: Optional[rospy.Time] = None
    _sub_model_states: Optional[rospy.Subscriber] = None
    _svc_reset_pose: Optional[rospy.ServiceProxy] = None
    _svc_reset_ball: Optional[rospy.ServiceProxy] = None
    _svc_set_robot_position: Optional[rospy.ServiceProxy] = None

    @classmethod
    def setUpClass(cls) -> None:
        super().setUpClass()

    def setUp(self) -> None:
        super().setUp()
        self._svc_reset_pose = rospy.ServiceProxy("/reset_pose", std_srvs.srv.Empty)
        self._svc_reset_ball = rospy.ServiceProxy("/reset_ball", std_srvs.srv.Empty)
        self._svc_set_robot_position = rospy.ServiceProxy("/set_robot_position", bitbots_msgs.srv.SetRobotPose)

        self.wait_for_simulator()
        self.reset_simulation()

        self._sub_model_states = rospy.Subscriber("/model_states", gazebo_msgs.msg.ModelStates,
                                                  callback=self._model_state_cb, queue_size=1)
        self._latest_model_states = None
        self._latest_model_states_time = None
        self.wait_for_model_state_update()

    def tearDown(self) -> None:
        super().tearDown()
        self._sub_model_states.unregister()

        self._svc_reset_pose.close()
        self._svc_reset_ball.close()
        self._svc_set_robot_position.close()

    def _model_state_cb(self, model_states: gazebo_msgs.msg.ModelStates):
        self._latest_model_states = model_states
        self._latest_model_states_time = rospy.Time.now()

    def reset_simulation(self):
        # TODO Call the /reset service after it has been fixed
        # rospy.ServiceProxy("/reset", std_srvs.srv.Empty)()

        self._svc_reset_pose(std_srvs.srv.EmptyRequest())
        self._svc_reset_ball(std_srvs.srv.EmptyRequest())

    def wait_for_simulator(self, timeout: float = 120):
        """
        Wait until the webots supervisor node is running because this implies that webots is running

        :param timeout: Timout in seconds after which this method aborts

        :raise TimeoutError: when aborting after *timeout* has expired
        """
        start_time = time.time()
        while True:
            try:
                remaining_timeout = timeout - (time.time() - start_time)
                self.assertNodeRunning(self._SUPERVISOR_NODE_NAME)
                self._svc_reset_pose.wait_for_service(timeout=remaining_timeout)
                self._svc_reset_ball.wait_for_service(timeout=remaining_timeout)
                self._svc_set_robot_position.wait_for_service(timeout=remaining_timeout)
                break
            except AssertionError:
                if start_time + timeout < time.time():
                    raise TimeoutError("timed out waiting for the simulator supervisor node")
                time.sleep(0.01)

    def wait_for_model_state_update(self, timeout: float = 2, num_updates: int = 1):
        """
        Wait until new model states have been received.

        This is useful to do after doing some action in the simulator before calling `assert*` methods so that
        the assertions work on the new data.

        :param timeout: Timout in seconds after which this method aborts
        :param num_updates: How many updates to wait for

        :raise TimeoutError: when aborting after *timeout* has expired
        """
        start_time = time.time()
        n = 0
        while n < num_updates:
            n += 1

            # if we have not received any updates yet, we wait until any one has been received at all
            if self._latest_model_states_time is None:
                while self._latest_model_states_time is None:
                    time.sleep(0.01)
                    if start_time + timeout < time.time():
                        raise TimeoutError("timed out waiting for new model states")

            # if we have already received an update, we wait until there is a newer one
            else:
                model_state_start_time = self._latest_model_states_time
                while not self._latest_model_states_time > model_state_start_time:
                    time.sleep(0.01)
                    if start_time + timeout < time.time():
                        raise TimeoutError("timed out waiting for new model states")

    def set_robot_position(self, position: Optional[geometry_msgs.msg.Point] = None, robot_name: str = "amy"):
        """
        Set the robot position in simulator

        :param robot_name: Name of the robot whose position should be set.
            Defaults to amy which is the only robot in single-robot simulations
        :param position: Position to which the robot should be teleported.
            If None, resets the robot to its original pose
        """
        if position:
            self._svc_set_robot_position(robot_name, position)
        else:
            self._svc_reset_pose(std_srvs.srv.EmptyRequest())

        self.wait_for_model_state_update(num_updates=2)

    def get_robot_pose(self, robot_name: str = "amy") -> geometry_msgs.msg.Pose:
        """
        Return the current pose of a robot.
        This is the *correct* pose as published by the simulator and not a result which stems from components like
        localization.

        :param robot_name: Name of the robot whose pose should be returned.
            Defaults to amy which is the only robot in single-robot simulations

        :raise AttributeError: if the pose lookup is not successful
        """
        if robot_name not in self._latest_model_states.name:
            raise AttributeError(f"no model states have been received for {robot_name}")

        i = self._latest_model_states.name.index(robot_name)
        return self._latest_model_states.pose[i]

    def assertRobotPosition(self, position: geometry_msgs.msg.Point, robot_name: str = "amy", *, threshold: int = 0.5, x_threshold: int = None, y_threshold: int = None, z_threshold: int = None):
        """
        Assert that a robot is at the specified position or at least close to it.

        :param position: Absolute position in webots coordinates at which the robot should be.
        :param robot_name: The robot of which the position should be verified.
            Defaults to amy which is the only robot in single-robot simulations
        :param threshold: Threshold which defines the amount of allowed derivation in meters from the specified position.
            By default, this means equal allowed derivation in all three axes. Each axis can be overwritten by the
            axis-specific argument.
        :param x_threshold: Maximum allowed derivation from the position on the x axis
        :param y_threshold: Maximum allowed derivation from the position on the y axis
        :param z_threshold: Maximum allowed derivation from the position on the z axis
        """
        x_threshold = x_threshold if x_threshold else threshold
        y_threshold = y_threshold if y_threshold else threshold
        z_threshold = z_threshold if z_threshold else threshold
        real_position = self.get_robot_pose(robot_name).position

        try:
            self.assertInRange(position.x, (real_position.x - x_threshold, real_position.x + x_threshold))
            self.assertInRange(position.y, (real_position.y - y_threshold, real_position.y + y_threshold))
            self.assertInRange(position.z, (real_position.z - z_threshold, real_position.z + z_threshold))
        except AssertionError:
            raise AssertionError(f"robot is not at (or close to) position {position}")
