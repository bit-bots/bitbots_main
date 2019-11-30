import rospy
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from humanoid_league_localization.srv import reset_filter


class AbstractInitialize(AbstractActionElement):

    def __init__(self, blackboard, dsd, parameters=None):
        super(AbstractInitialize, self).__init__(blackboard, dsd, parameters=None)
        rospy.wait_for_service('reset_filter')

        self.called = False
        self.last_service_call = 0
        self.time_between_calls = 2  # [s]

    def perform(self, reevaluate=False):
        raise NotImplementedError

class DoNothing(AbstractInitialize):
    def perform(self, reevaluate=False):
        rospy.loginfo("doing nothing")
        return

class InitPose(AbstractInitialize):
    def perform(self, reevaluate=False):
        rospy.loginfo("initializing pose")
        rospy.wait_for_service('reset_filter')
        reset_filter_prox = rospy.ServiceProxy('reset_filter', reset_filter)
        try:
            resp = reset_filter_prox(0, None, None)
            return resp.success
        except rospy.ServiceException as e:
            print
            "Service call failed: %s" % e


class InitLeftHalf(AbstractInitialize):
    def perform(self, reevaluate=False):
        rospy.loginfo("initializing left half")
        rospy.wait_for_service('reset_filter')
        reset_filter_prox = rospy.ServiceProxy('reset_filter', reset_filter)
        try:
            resp = reset_filter_prox(1, None, None)
            return resp.success
        except rospy.ServiceException as e:
            print
            "Service call failed: %s" % e


class InitRightHalf(AbstractInitialize):
    def perform(self, reevaluate=False):
        rospy.loginfo("initializing right half")

        rospy.wait_for_service('reset_filter')
        reset_filter_prox = rospy.ServiceProxy('reset_filter', reset_filter)
        try:
            resp = reset_filter_prox(2, None, None)
            return resp.success
        except rospy.ServiceException as e:
            print
            "Service call failed: %s" % e


class InitPosition(AbstractInitialize):
    def perform(self, reevaluate=False):
        rospy.loginfo("initializing position")

        rospy.wait_for_service('reset_filter')
        reset_filter_prox = rospy.ServiceProxy('reset_filter', reset_filter)
        try:
            resp = reset_filter_prox(3, 0, 0)
            return resp.success
        except rospy.ServiceException as e:
            print
            "Service call failed: %s" % e

class InitSet(AbstractActionElement):
    def perform(self, reevaluate=False):
        return NotImplementedError

class InitPlaying(AbstractActionElement):

        def perform(self, reevaluate=False):
            return NotImplementedError

class InitPenalty(AbstractActionElement):
    def perform(self, reevaluate=False):
        return NotImplementedError

