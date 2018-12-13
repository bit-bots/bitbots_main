from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class BallClose(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(BallClose, self).__init__(blackboard, dsd, parameters)
        self.max_kick_distance = self.blackboard.config['max_kick_distance']

    def perform(self, reevaluate=False):
        if self.blackboard.world_model.get_ball_distance() < self.max_kick_distance:
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True
