import rospy
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from bitbots_localization.srv import reset_filter


class AbstractInitialize(AbstractActionElement):

    def __init__(self, blackboard, dsd, parameters=None):
        super(AbstractInitialize, self).__init__(blackboard, dsd, parameters=None)

        self.called = False
        self.last_service_call = 0
        self.time_between_calls = 2  # [s]

        self.first_perform = True

    def perform(self, reevaluate=False):
        raise NotImplementedError

class DoNothing(AbstractInitialize):
    def perform(self, reevaluate=False):
        rospy.logdebug("doing nothing")
        return

class InitPose(AbstractInitialize):
    def perform(self, reevaluate=False):
        rospy.logdebug("initializing pose")
        rospy.wait_for_service('bitbots_localization/reset_filter')
        reset_filter_proxy = rospy.ServiceProxy('bitbots_localization/reset_filter', reset_filter)
        try:
            resp = reset_filter_proxy(0, None, None)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


class InitLeftHalf(AbstractInitialize):
    def perform(self, reevaluate=False):
        rospy.logdebug("initializing left half")
        rospy.wait_for_service('bitbots_localization/reset_filter')
        reset_filter_proxy = rospy.ServiceProxy('bitbots_localization/reset_filter', reset_filter)
        try:
            resp = reset_filter_proxy(1, None, None)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


class InitRightHalf(AbstractInitialize):
    def perform(self, reevaluate=False):
        rospy.logdebug("initializing right half")

        rospy.wait_for_service('bitbots_localization/reset_filter')
        reset_filter_proxy = rospy.ServiceProxy('bitbots_localization/reset_filter', reset_filter)
        try:
            resp = reset_filter_proxy(2, None, None)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


class InitPosition(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        rospy.logdebug("initializing position")

        rospy.wait_for_service('bitbots_localization/reset_filter')
        reset_filter_proxy = rospy.ServiceProxy('bitbots_localization/reset_filter', reset_filter)
        try:
            resp = reset_filter_proxy(
                3, 
                self.blackboard.poseX,
                self.blackboard.poseY)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return self.pop()


class InitSide(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        rospy.logdebug("initializing on the side line of our half")

        rospy.wait_for_service('bitbots_localization/reset_filter')
        reset_filter_proxy = rospy.ServiceProxy('bitbots_localization/reset_filter', reset_filter)
        try:
            resp = reset_filter_proxy(0, None, None)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return self.pop()
