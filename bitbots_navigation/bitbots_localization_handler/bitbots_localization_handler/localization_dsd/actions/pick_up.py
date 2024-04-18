from bitbots_localization_handler.localization_dsd.actions import AbstractLocalizationActionElement


class WaitForPickupEnd(AbstractLocalizationActionElement):
    def perform(self, reevaluate=False):
        if not self.blackboard.picked_up():
            self.pop()
