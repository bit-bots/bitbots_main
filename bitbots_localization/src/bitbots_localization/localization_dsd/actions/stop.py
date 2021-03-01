import rospy
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from bitbots_localization.srv import set_paused


class AbstractLocalizationPause(AbstractActionElement):

    def __init__(self, blackboard, dsd, parameters=None):
        super(AbstractLocalizationPause, self).__init__(blackboard, dsd, parameters=parameters)
        self.stop_filter_prox = rospy.ServiceProxy('pause_localization', set_paused)

    def set_paused(self, paused):
        self.do_not_reevaluate()
        rospy.wait_for_service('pause_localization')
        try:
            resp = self.stop_filter_prox(paused)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


class LocalizationStop(AbstractLocalizationPause):
    def __init__(self, blackboard, dsd, parameters=None):
        super(LocalizationStop, self).__init__(blackboard, dsd, parameters=parameters)
    
    def perform(self, reevaluate=False):
        rospy.logdebug("Stop localization")
        self.set_paused(True)
        return self.pop()


class LocalizationStart(AbstractLocalizationPause):
    def __init__(self, blackboard, dsd, parameters=None):
        super(LocalizationStart, self).__init__(blackboard, dsd, parameters=parameters)
        
    def perform(self, reevaluate=False):
        rospy.logdebug("Start localization")
        self.set_paused(False)
        return self.pop()
        
