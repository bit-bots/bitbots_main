import rospy
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class SetFootZero(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(SetFootZero, self).__init__(blackboard, dsd, parameters)
        self.first_perform = True

    def perform(self, reevaluate=False):
        if self.first_perform:
            # Executing this once is sufficient
            self.first_perform = False
            try:
                self.blackboard.foot_zero_service.wait_for_service(0.5)
                self.blackboard.foot_zero_service()
            except rospy.ROSException:
                rospy.logwarn("No foot zeroing service accessible, will not reset sensors")

            self.pop()
