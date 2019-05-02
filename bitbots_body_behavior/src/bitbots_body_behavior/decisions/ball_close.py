from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class BallClose(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(BallClose, self).__init__(blackboard, dsd, parameters)
        self.ball_close_distance = self.blackboard.config['ball_close_distance']

    def perform(self, reevaluate=False):
        if self.blackboard.world_model.get_ball_distance() < self.ball_close_distance:
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True
