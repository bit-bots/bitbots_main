import rclpy
import tf2_ros as tf2
from rclpy.duration import Duration
from rclpy.time import Time
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from bitbots_localization.srv import ResetFilter


class AbstractInitialize(AbstractActionElement):

    def __init__(self, blackboard, dsd, parameters=None):
        super(AbstractInitialize, self).__init__(blackboard, dsd, parameters)

        # Save the type the instance that called super, so we know what was the last init mode
        if not isinstance(self, RedoLastInit):
            blackboard.last_init_action_type = self.__class__
            try:
                blackboard.last_init_odom_transform = self.blackboard.tf_buffer.lookup_transform(
                    blackboard.odom_frame,
                    blackboard.base_footprint_frame,
                    Time(seconds=0, nanoseconds=0),
                    Duration(seconds=1.0))  # wait up to 1 second for odom data
            except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
                self.blackboard.node.get_logger().warn(f"Not able to save the odom position due to a tf error: {e}")
            self.blackboard.node.get_logger().info("Set last init action type to ", blackboard.last_init_action_type)

        self.called = False
        self.last_service_call = 0
        self.time_between_calls = 2  # [s]

        self.first_perform = True

    def perform(self, reevaluate=False):
        raise NotImplementedError


class InitLeftHalf(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing on the left half")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info('Localization reset service not available, waiting again...')
        self.blackboard.reset_filter_proxy(1, None, None, None)
        return self.pop()


class InitRightHalf(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing on the right half")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info('Localization reset service not available, waiting again...')
        self.blackboard.reset_filter_proxy(2, None, None, None)
        return self.pop()


class InitPosition(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing position")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info('Localization reset service not available, waiting again...')
        self.blackboard.reset_filter_proxy(3, self.blackboard.poseX, self.blackboard.poseY, None)
        return self.pop()


class InitSide(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing on the side lines of our side")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info('Localization reset service not available, waiting again...')
        self.blackboard.reset_filter_proxy(0, None, None, None)
        return self.pop()


class InitGoal(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing on the side lines of our side")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.reset_filter_proxy.node.get_logger().info('Localization reset service not available, waiting again...')
        self.blackboard.reset_filter_proxy(4, -self.blackboard.field_length / 2, 0)
        return self.pop()


class InitPenaltyKick(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing behind penalty mark")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info('Localization reset service not available, waiting again...')
        self.blackboard.reset_filter_proxy(4, self.blackboard.field_length / 2 - 2, 0)
        return self.pop()


class RedoLastInit(AbstractInitialize):
    """
    Executes an action with the type of the last action
    """
    def __init__(self, blackboard, dsd, parameters=None):
        super(RedoLastInit, self).__init__(blackboard, dsd, parameters)
        # Creates an instance of the last init action
        self.sub_action = blackboard.last_init_action_type(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.get_logger().debug("Redoing the last init")
        return self.sub_action.perform(reevaluate)
