from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from std_msgs.msg import Bool


class DeactivateHCM(AbstractActionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.blackboard.misc.hcm_deactivate_pub.publish(Bool(data=True))
        self.pop()
