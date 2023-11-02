import tf2_ros as tf2
from rclpy.duration import Duration
from rclpy.time import Time

from bitbots_localization.srv import ResetFilter
from bitbots_localization_handler.localization_dsd.actions import AbstractLocalizationActionElement


class AbstractInitialize(AbstractLocalizationActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

        # Save the type the instance that called super, so we know what was the last init mode
        if not isinstance(self, RedoLastInit):
            self.blackboard.last_init_action_type = self.__class__
            try:
                # only need this in simulation
                if self.blackboard.use_sim_time:
                    self.blackboard.last_init_odom_transform = self.blackboard.tf_buffer.lookup_transform(
                        self.blackboard.odom_frame,
                        self.blackboard.base_footprint_frame,
                        Time(seconds=0, nanoseconds=0),
                        Duration(seconds=1.0),
                    )  # wait up to 1 second for odom data
            except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
                self.blackboard.node.get_logger().warn(f"Not able to save the odom position due to a tf error: {e}")
            self.blackboard.node.get_logger().info(
                f"Set last init action type to {self.blackboard.last_init_action_type}"
            )

    def perform(self, reevaluate=False):
        raise NotImplementedError


class InitLeftHalf(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing on the left half")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info("Localization reset service not available, waiting again...")
        self.blackboard.reset_filter_proxy.call_async(ResetFilter.Request(init_mode=ResetFilter.Request.LEFT_HALF))
        return self.pop()


class InitRightHalf(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing on the right half")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info("Localization reset service not available, waiting again...")
        self.blackboard.reset_filter_proxy.call_async(ResetFilter.Request(init_mode=ResetFilter.Request.RIGHT_HALF))
        return self.pop()


class InitPosition(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing position")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info("Localization reset service not available, waiting again...")
        self.blackboard.reset_filter_proxy.call_async(
            ResetFilter.Request(
                init_mode=ResetFilter.Request.POSITION, x=self.blackboard.poseX, y=self.blackboard.poseY
            )
        )
        return self.pop()


class InitSide(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing on the side lines of our side")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info("Localization reset service not available, waiting again...")
        self.blackboard.reset_filter_proxy.call_async(ResetFilter.Request(init_mode=ResetFilter.Request.START_RIGHT))
        return self.pop()


class InitGoal(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing on the side lines of our side")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.reset_filter_proxy.node.get_logger().info(
                "Localization reset service not available, waiting again..."
            )
        self.blackboard.reset_filter_proxy.call_async(
            ResetFilter.Request(init_mode=ResetFilter.Request.POSE, x=float(-self.blackboard.field_length / 2), y=0.0)
        )
        return self.pop()


class InitPenaltyKick(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing behind penalty mark")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info("Localization reset service not available, waiting again...")
        self.blackboard.reset_filter_proxy.call_async(
            ResetFilter.Request(
                init_mode=ResetFilter.Request.POSE, x=float(self.blackboard.field_length / 2 - 2), y=0.0
            )
        )
        return self.pop()


class RedoLastInit(AbstractInitialize):
    """
    Executes an action with the type of the last action
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        # Creates an instance of the last init action
        self.sub_action: AbstractInitialize = self.blackboard.last_init_action_type(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.blackboard.node.get_logger().debug("Redoing the last init")
        return self.sub_action.perform(reevaluate)
