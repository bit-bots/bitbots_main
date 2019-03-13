from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class BallDangerous(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(BallDangerous, self).__init__(blackboard, dsd, parameters)
        self.ball_dangerous_goal_radius = self.blackboard.config['ball_dangerous_goal_radius']

    def perform(self, reevaluate=False):
        if self._in_dangerous_area(self.blackboard.world_model.get_ball_position_xy()):
            return 'YES'
        return 'NO'

    def _in_dangerous_area(self, position):
        """"
        returns whether the position is in the dangerous area (close to the goal)
        """

        # close enough on the x axis
        if position[0] > -(self.blackboard.field_length / 2) + self.ball_dangerous_goal_radius:
            return False

        if (-self.blackboard.goal_width / 2) - self.ball_dangerous_goal_radius <= \
                position[1] <= \
                (self.blackboard.goal_width / 2) + self.ball_dangerous_goal_radius:
            return True
        return False



    def get_reevaluate(self):
        return True

