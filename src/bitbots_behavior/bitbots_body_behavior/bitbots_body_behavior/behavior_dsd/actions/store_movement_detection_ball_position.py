from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class StoreBallMovementDetectionStartPosition(AbstractActionElement):
    """
    Stores the starting position for ball movement detection.
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters: dict):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.blackboard.misc.ball_movement_detection_start_ball_position = (
            self.blackboard.world_model.get_ball_position_xy()
        )
        self.pop()


class ForgetBallStartPosition(AbstractActionElement):
    """
    Forgets the starting position for ball movement detection.
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters: dict):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.blackboard.misc.ball_movement_detection_start_ball_position = None
        self.pop()
