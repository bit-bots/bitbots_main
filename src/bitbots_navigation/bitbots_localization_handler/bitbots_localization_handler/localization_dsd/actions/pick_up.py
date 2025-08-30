from bitbots_localization_handler.localization_dsd.actions import AbstractLocalizationActionElement


class WaitForPickupEnd(AbstractLocalizationActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.history = []

    def perform(self, reevaluate=False):
        self.history.append(self.blackboard.picked_up())
        if len(self.history) > 100 and not any(self.history[-100:]):
            self.pop()
