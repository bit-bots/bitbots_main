import rospy
import tf2_ros as tf2
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from bitbots_localization.srv import ResetFilter


class AbstractInitialize(AbstractActionElement):

    def __init__(self, blackboard, dsd, parameters=None):
        super(AbstractInitialize, self).__init__(blackboard, dsd, parameters)

        # Save the type the instance that called super, so we know what was the last init mode
        if not isinstance(self, RedoLastInit):
            blackboard.last_init_action_type = self.__class__
            try:
                blackboard.last_init_odom_transform = self.blackboard.tf_buffer.lookup_transform(
                    blackboard.odom_frame,
                    blackboard.base_footprint_frame,
                    rospy.Time(0))
            except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
                rospy.logerr(f"Not able to save the odom position due to a tf error: {e}")
            print("Set last init action type to ", blackboard.last_init_action_type)

        self.called = False
        self.last_service_call = 0
        self.time_between_calls = 2  # [s]

        self.first_perform = True

    def perform(self, reevaluate=False):
        raise NotImplementedError


class InitPose(AbstractInitialize):
    def perform(self, reevaluate=False):
        rospy.logdebug("initializing pose")
        rospy.wait_for_service('reset_localization')
        reset_filter_proxy = rospy.ServiceProxy('reset_localization', ResetFilter)
        try:
            resp = reset_filter_proxy(0, None, None, None)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return self.pop()


class InitLeftHalf(AbstractInitialize):
    def perform(self, reevaluate=False):
        rospy.logdebug("initializing left half")
        rospy.wait_for_service('reset_localization')
        reset_filter_proxy = rospy.ServiceProxy('reset_localization', ResetFilter)
        try:
            resp = reset_filter_proxy(1, None, None, None)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return self.pop()


class InitRightHalf(AbstractInitialize):
    def perform(self, reevaluate=False):
        rospy.logdebug("initializing right half")

        rospy.wait_for_service('reset_localization')
        reset_filter_proxy = rospy.ServiceProxy('reset_localization', ResetFilter)
        try:
            resp = reset_filter_proxy(2, None, None, None)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return self.pop()


class InitPosition(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        rospy.logdebug("initializing position")

        rospy.wait_for_service('reset_localization')
        reset_filter_proxy = rospy.ServiceProxy('reset_localization', ResetFilter)
        try:
            resp = reset_filter_proxy(
                3,
                self.blackboard.poseX,
                self.blackboard.poseY,
                None)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return self.pop()


class InitSide(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        rospy.logdebug("initializing on the side line of our half")

        rospy.wait_for_service('reset_localization')
        reset_filter_proxy = rospy.ServiceProxy('reset_localization', ResetFilter)
        try:
            resp = reset_filter_proxy(0, None, None, None)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return self.pop()


class InitGoal(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        rospy.logdebug("initializing in the center of our goal")

        rospy.wait_for_service('reset_localization')
        reset_filter_proxy = rospy.ServiceProxy('reset_localization', ResetFilter)
        try:
            resp = reset_filter_proxy(4, -rospy.get_param('field_length', 9) / 2, 0, 0)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return self.pop()


class InitPenaltyKick(AbstractInitialize):
    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        rospy.logdebug("initializing behind penalty mark")

        rospy.wait_for_service('reset_localization')
        reset_filter_proxy = rospy.ServiceProxy('reset_localization', ResetFilter)
        try:
            resp = reset_filter_proxy(4, rospy.get_param('field_length', 9) / 2 - 2, 0, 0)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        return self.pop()


class RedoLastInit(AbstractInitialize):
    """
    Executes an action with the type of the last action
    """
    def __init__(self, blackboard, dsd, parameters=None):
        super(RedoLastInit, self).__init__(blackboard, dsd, parameters)
        # Creates an instance of the last init action
        self.sub_action = blackboard.last_init_action_type(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        rospy.logdebug("redoing the last init")
        return self.sub_action.perform(reevaluate)