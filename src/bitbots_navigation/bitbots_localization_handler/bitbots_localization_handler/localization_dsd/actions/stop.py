from bitbots_localization.srv import SetPaused

from bitbots_localization_handler.localization_dsd.actions import AbstractLocalizationActionElement


class AbstractLocalizationPause(AbstractLocalizationActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters=parameters)

    def set_paused(self, paused):
        self.do_not_reevaluate()
        while not self.blackboard.stop_filter_proxy.wait_for_service(timeout_sec=3.0):
            self.blackboard.node.get_logger().info("Localization reset service not available, waiting again...")
        self.blackboard.stop_filter_proxy.call_async(SetPaused.Request(paused=paused))


class LocalizationStop(AbstractLocalizationPause):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters=parameters)

    def perform(self, reevaluate=False):
        self.blackboard.node.get_logger().debug("Stop localization")
        self.set_paused(True)
        return self.pop()


class LocalizationStart(AbstractLocalizationPause):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters=parameters)

    def perform(self, reevaluate=False):
        self.blackboard.node.get_logger().debug("Start localization")
        self.set_paused(False)
        return self.pop()


class DoNothing(AbstractLocalizationActionElement):
    def perform(self, reevaluate=False):
        self.blackboard.node.get_logger().debug("Doing nothing")
        return
