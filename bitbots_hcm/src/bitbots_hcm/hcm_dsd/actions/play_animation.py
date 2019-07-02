import rospy
import actionlib
import humanoid_league_msgs.msg
import bitbots_msgs.msg
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard, STATE_GETTING_UP


class AbstractPlayAnimation(AbstractActionElement):
    """
    Abstract class to create actions for playing animations
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(AbstractPlayAnimation, self).__init__(blackboard, dsd, parameters=None)

        self.first_perform = True

    def perform(self, reevaluate=False):
        # we never want to leave the action when we play an animation
        # deactivate the reevaluate
        if not self.blackboard.shut_down_request:
            self.do_not_reevaluate()

        if self.first_perform:
            # get the animation that should be played
            # defined by implementations of this abstract class
            anim = self.chose_animation()

            # try to start animation
            sucess = self.start_animation(anim)            
            # if we fail, we need to abort this action
            if not sucess:
                rospy.logerr("Could not start animation. Will abort play animation action!")
                return self.pop()

            self.first_perform = False
            return

        if self.animation_finished():
            # we are finished playing this animation
            return self.pop()

    def chose_animation(self):
        # this is what has to be implemented returning the animation to play
        raise NotImplementedError

    def start_animation(self, anim):
        """
        This will NOT wait by itself. You have to check
        animation_finished()
        by yourself.
        :param anim: animation to play
        :return:
        """

        rospy.loginfo("Playing animation " + anim)
        if anim is None or anim == "":
            rospy.logwarn("Tried to play an animation with an empty name!")
            return False
        first_try = self.blackboard.animation_action_client.wait_for_server(
            rospy.Duration(rospy.get_param("hcm/anim_server_wait_time", 1)))     
        if not first_try:
            server_running = False
            while not server_running and not self.blackboard.shut_down_request and not rospy.is_shutdown():            
                rospy.logerr_throttle(5.0,
                "Animation Action Server not running! Motion can not work without animation action server. "
                "Will now wait until server is accessible!")
                server_running = self.blackboard.animation_action_client.wait_for_server(rospy.Duration(1))
            if server_running:
                rospy.logwarn("Animation server now running, hcm will go on.")
            else:               
                rospy.logwarn("Animation server did not start.")
                return False
        goal = humanoid_league_msgs.msg.PlayAnimationGoal()
        goal.animation = anim
        goal.hcm = True  # the animation is from the hcm
        self.blackboard.animation_action_client.send_goal(goal)
        return True

    def animation_finished(self):
        state = self.blackboard.animation_action_client.get_state()
        return state == 3


class PlayAnimationStandUpFront(AbstractPlayAnimation):
    def chose_animation(self):
        rospy.loginfo("PLAYING STAND UP FRONT ANIMATION")
        return self.blackboard.stand_up_front_animation


class PlayAnimationStandUpBack(AbstractPlayAnimation):
    def chose_animation(self):
        rospy.loginfo("PLAYING STAND UP BACK ANIMATION")
        return self.blackboard.stand_up_back_animation

class PlayAnimationStandUpLeft(AbstractPlayAnimation):
    def chose_animation(self):
        rospy.loginfo("PLAYING STAND UP LEFT ANIMATION")
        return self.blackboard.stand_up_left_animation

class PlayAnimationStandUpRight(AbstractPlayAnimation):
    def chose_animation(self):
        rospy.loginfo("PLAYING STAND UP RIGHT ANIMATION")
        return self.blackboard.stand_up_right_animation


class PlayAnimationFallingLeft(AbstractPlayAnimation):
    def chose_animation(self):
        rospy.loginfo("PLAYING FALLING LEFT ANIMATION")
        return self.blackboard.falling_animation_left


class PlayAnimationFallingRight(AbstractPlayAnimation):
    def chose_animation(self):
        rospy.loginfo("PLAYING FALLING RIGHT ANIMATION")
        return self.blackboard.falling_animation_right


class PlayAnimationFallingFront(AbstractPlayAnimation):
    def chose_animation(self):
        rospy.loginfo("PLAYING FALLING FRONT ANIMATION")
        return self.blackboard.falling_animation_front


class PlayAnimationFallingBack(AbstractPlayAnimation):
    def chose_animation(self):
        rospy.loginfo("PLAYING FALLING BACK ANIMATION")
        return self.blackboard.falling_animation_back


class PlayAnimationPenalty(AbstractPlayAnimation):
    def chose_animation(self):
        return self.blackboard.penalty_animation


class PlayAnimationWalkready(AbstractPlayAnimation):
    def chose_animation(self):
        return self.blackboard.walkready_animation


class PlayAnimationSitDown(AbstractPlayAnimation):
    def chose_animation(self):
        return self.blackboard.sit_down_animation


class PlayAnimationMotorOff(AbstractPlayAnimation):
    def chose_animation(self):
        return self.blackboard.motor_off_animation


class PlayAnimationDynup(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(PlayAnimationDynup, self).__init__(blackboard, dsd, parameters=None)

        self.first_perform = True

    def perform(self, reevaluate=False):
        # we never want to leave the action when we play an animation
        # deactivate the reevaluate
        if not self.blackboard.shut_down_request:
            self.do_not_reevaluate()

        if self.first_perform:
            # get the animation that should be played
            # defined by implementations of this abstract class

            # try to start animation
            success = self.start_animation()
            # if we fail, we need to abort this action
            if not success:
                rospy.logerr("Could not start animation. Will abort play animation action!")
                return self.pop()

            self.first_perform = False
            return

        if self.animation_finished():
            # we are finished playing this animation
            return self.pop()

    def chose_animation(self):
        # this is what has to be implemented returning the animation to play
        raise NotImplementedError

    def start_animation(self):
        """
        This will NOT wait by itself. You have to check
        animation_finished()
        by yourself.
        :return:
        """

        first_try = self.blackboard.dynup_action_client.wait_for_server(
            rospy.Duration(rospy.get_param("hcm/anim_server_wait_time", 1)))
        if not first_try:
            server_running = False
            while not server_running and not self.blackboard.shut_down_request and not rospy.is_shutdown():
                rospy.logerr_throttle(5.0,
                                      "Dynup Action Server not running! Dynup cannot work without dynup server!"
                                      "Will now wait until server is accessible!")
                server_running = self.blackboard.dynup_action_client.wait_for_server(rospy.Duration(1))
            if server_running:
                rospy.logwarn("Dynup server now running, hcm will go on.")
            else:
                rospy.logwarn("Dynup server did not start.")
                return False
        goal = bitbots_msgs.msg.DynUpGoal()
        goal.front = True  # is currently ignored
        self.blackboard.dynup_action_client.send_goal(goal)
        return True

    def animation_finished(self):
        state = self.blackboard.dynup_action_client.get_state()
        return state == 3

