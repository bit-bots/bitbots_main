import math

import tf2_ros as tf2
from bitbots_localization.srv import ResetFilter
from rclpy.duration import Duration
from rclpy.time import Time

from bitbots_localization_handler.localization_dsd.actions import AbstractLocalizationActionElement


class AbstractInitialize(AbstractLocalizationActionElement):
    def __init__(self, blackboard, dsd, parameters):
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
                        Time(),
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
                init_mode=ResetFilter.Request.POSITION,
                x=self.blackboard.robot_pose.pose.pose.position.x,
                y=self.blackboard.robot_pose.pose.pose.position.y,
            )
        )
        return self.pop()


class InitPoseAfterFall(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing at pose after fall")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info("Localization reset service not available, waiting again...")

        # Calculate the angle of the robot after the fall by adding the yaw difference during the fall
        # (estimated by the IMU) to the last known localization yaw
        esimated_angle = (
            self.blackboard.get_imu_yaw()
            - self.blackboard.imu_yaw_before_fall
            + self.blackboard.get_localization_yaw()
            + math.tau
        ) % math.tau

        # Tell the localization to reset to the last position and the estimated angle
        self.blackboard.reset_filter_proxy.call_async(
            ResetFilter.Request(
                init_mode=ResetFilter.Request.POSE,
                x=self.blackboard.robot_pose.pose.pose.position.x,
                y=self.blackboard.robot_pose.pose.pose.position.y,
                angle=esimated_angle,
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

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        # Creates an instance of the last init action
        self.sub_action: AbstractInitialize = self.blackboard.last_init_action_type(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.blackboard.node.get_logger().debug("Redoing the last init")
        return self.sub_action.perform(reevaluate)


class StoreCurrentIMUYaw(AbstractLocalizationActionElement):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Storing current IMU yaw")
        # Get yaw component of the current orientation before the fall
        # It drifts over time, so this is not an absolute measurement, but it will help to overcome
        # The short time during the fall at which we have no odometry
        self.blackboard.imu_yaw_before_fall = self.blackboard.get_imu_yaw()
        return self.pop()
