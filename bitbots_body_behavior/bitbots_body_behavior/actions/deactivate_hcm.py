from actionlib_msgs.msg import GoalID
from dynamic_stack_decider import AbstractActionElement
from humanoid_league_msgs.msg import Strategy
from std_msgs.msg import Bool


class DeactivateHCM(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.blackboard.hcm_deactivate_pub.publish(Bool(True))
        self.pop()
