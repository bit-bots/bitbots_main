from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class BallClose(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(BallClose, self).__init__(blackboard, dsd, parameters)
        self.ball_close_distance = parameters.get("distance", self.blackboard.config['ball_close_distance'])

    def perform(self, reevaluate=False):
        """
        Determines whether the ball is in close range to the robot. The distance threshold is set in the config file.
        :param reevaluate:
        :return:
        """
        self.publish_debug_data("ball_distance", self.blackboard.world_model.get_ball_distance())

        if self.blackboard.world_model.get_ball_distance() < self.ball_close_distance:
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True
