from rclpy.time import Time
from rclpy.duration import Duration
from bitbots_blackboard.blackboard import BodyBlackboard

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class LocalizationAvailable(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super(LocalizationAvailable, self).__init__(blackboard, dsd, parameters)
        self.use_localization = blackboard.config["use_localization"]  # type: bool

    def perform(self, reevaluate=False):
        """
        Determines whether the localization should be used in the current situation.
        :param reevaluate:
        :return:
        """
        if self.use_localization and self.blackboard.world_model.localization_pose_current():
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True


class LocalizationPrecision(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super(LocalizationPrecision, self).__init__(blackboard, dsd, parameters)
        self.debounce = Duration(seconds=parameters.get("debounce", 0.0))
        self.last_decision = "LOW"
        self.last_decision_time: Time = Time(seconds=0, clock_type=self.blackboard.node.get_clock().clock_type)

    def perform(self, reevaluate=False):
        """
        Determines whether the localization should be used in the current situation.
        :param reevaluate:
        :return:
        """
        current_time = self.blackboard.node.get_clock().now()
        last_decision_before_debounce: bool = current_time - self.last_decision_time > self.debounce

        if last_decision_before_debounce:
            if self.blackboard.world_model.localization_precision_in_threshold():
                self.last_decision = "HIGH"
            self.last_decision = "LOW"

            self.last_decision_time = current_time
            self.blackboard.node.get_logger().warn(f"made new decision was {self.last_decision}")

        self.blackboard.node.get_logger().warn(f"returning {self.last_decision}")
        return self.last_decision

    def get_reevaluate(self):
        return True
