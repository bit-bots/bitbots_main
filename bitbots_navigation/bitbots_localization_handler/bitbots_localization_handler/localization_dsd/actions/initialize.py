from bitbots_localization.srv import ResetFilter

from bitbots_localization_handler.localization_dsd.actions import AbstractLocalizationActionElement


class InitOpponentHalf(AbstractLocalizationActionElement):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing on the opponent half")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info("Localization reset service not available, waiting again...")
        self.blackboard.reset_filter_proxy.call_async(ResetFilter.Request(init_mode=ResetFilter.Request.OPPONENT_HALF))
        return self.pop()


class InitOwnHalf(AbstractLocalizationActionElement):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing on our half")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info("Localization reset service not available, waiting again...")
        self.blackboard.reset_filter_proxy.call_async(ResetFilter.Request(init_mode=ResetFilter.Request.OWN_HALF))
        return self.pop()


class InitPosition(AbstractLocalizationActionElement):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing position")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info("Localization reset service not available, waiting again...")

        # Abort if we don't know the current position
        if self.blackboard.robot_pose is None:
            self.blackboard.node.get_logger().warn(
                "Can't initialize position, because we don't know the current position"
            )
            return self.pop()

        # Reset the localization to the current position
        self.blackboard.reset_filter_proxy.call_async(
            ResetFilter.Request(
                init_mode=ResetFilter.Request.POSITION,
                x=self.blackboard.robot_pose.pose.pose.position.x,
                y=self.blackboard.robot_pose.pose.pose.position.y,
            )
        )
        return self.pop()


class InitSide(AbstractLocalizationActionElement):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing on the side lines of our side")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info("Localization reset service not available, waiting again...")
        self.blackboard.reset_filter_proxy.call_async(ResetFilter.Request(init_mode=ResetFilter.Request.OWN_SIDELINE))
        return self.pop()


class InitGoal(AbstractLocalizationActionElement):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        self.blackboard.node.get_logger().debug("Initializing on the side lines of our side")
        while not self.blackboard.reset_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info("Localization reset service not available, waiting again...")
        self.blackboard.reset_filter_proxy.call_async(
            ResetFilter.Request(init_mode=ResetFilter.Request.POSE, x=float(-self.blackboard.field_length / 2), y=0.0)
        )
        return self.pop()


class InitPenaltyKick(AbstractLocalizationActionElement):
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
