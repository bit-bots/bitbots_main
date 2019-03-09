from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class KickBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(KickBall, self).__init__(blackboard, dsd, parameters)
        self.right_kick = 'kick_right'  # TODO get actual name of parameter from some config

    def perform(self, reevaluate=False):
        if not self.blackboard.animation.is_animation_busy():
            self.blackboard.animation.play_animation(self.right_kick)


class KickBallRight(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(KickBallRight, self).__init__(blackboard, dsd, parameters)
        self.right_kick = 'kick_right'  # TODO get actual name of parameter from some config

    def perform(self, reevaluate=False):
        if not self.blackboard.animation.is_animation_busy():
            self.blackboard.animation.play_animation(self.right_kick)


class KickBallLeft(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(KickBallLeft, self).__init__(blackboard, dsd, parameters)
        self.right_kick = 'kick_left'  # TODO get actual name of parameter from some config

    def perform(self, reevaluate=False):
        if not self.blackboard.animation.is_animation_busy():
            self.blackboard.animation.play_animation(self.right_kick)


class KickBallVeryHard(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.right_hard_kick = 'kick_right_string'  # TODO get actual name of parameter from some config

    def perform(self, reevaluate=False):
        if not self.blackboard.animation.is_animation_busy():
            self.blackboard.animation.play_animation(self.right_hard_kick)
