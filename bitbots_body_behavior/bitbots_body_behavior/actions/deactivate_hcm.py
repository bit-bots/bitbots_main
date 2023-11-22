from dynamic_stack_decider import AbstractActionElement
from std_msgs.msg import Bool

from bitbots_blackboard.blackboard import BodyBlackboard


class DeactivateHCM(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.blackboard.misc.hcm_deactivate_pub.publish(Bool(data=True))
        self.pop()
