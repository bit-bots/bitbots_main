import rospy 
import actionlib
import humanoid_league_msgs.msg
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard, STATE_GETTING_UP


class AbstractPlayAnimation(AbstractActionElement):
    """
    Abstract class to create actions for playing animations
    """

    def __init__(self, blackboard, _):
        super(AbstractPlayAnimation, self).__init__(blackboard)
        self.first_perform = True

    def perform(self, blackboard, reevaluate=False):
        # we never want to leave the action when we play an animation
        # deactivate the reevaluate
        self.do_not_reevaluate()

        if self.first_perform:
            self.first_perform = False

            # get the animation that should be played
            # defined by implementations of this abstract class
            anim = self.chose_animation(blackboard)
            
            #start animation
            self.start_animation(blackboard, anim)
            return
        
        if self.animation_finished(blackboard):
            # we are finished playing this animation
            return self.pop()

    def chose_animation(self, blackboard):
        # this is what has to be implemented returning the animation to play
        raise NotImplementedError

    def chose_state_to_publish(self, blackboard):
        raise NotImplementedError

    def start_animation(self, blackboard, anim):
        """
        This will NOT wait by itself. You have to check
        animation_finished()
        by yourself.
        :param anim: animation to play
        :return:
        """
        blackboard.hcm_animation_playing = False  # will be set true when the hcm receives keyframe callback
        blackboard.hcm_animation_finished = False

        rospy.loginfo("Playing animation " + anim)
        if anim is None or anim == "":
            rospy.logwarn("Tried to play an animation with an empty name!")
            return False
        first_try = blackboard.animation_action_client.wait_for_server(
            rospy.Duration(rospy.get_param("hcm/anim_server_wait_time", 10)))
        if not first_try:
            rospy.logerr(
                "Animation Action Server not running! Motion can not work without animation action server. "
                "Will now wait until server is accessible!")
            blackboard.animation_action_client.wait_for_server()
            rospy.logwarn("Animation server now running, hcm will go on.")
        goal = humanoid_league_msgs.msg.PlayAnimationGoal()
        goal.animation = anim
        goal.hcm = True  # the animation is from the hcm
        blackboard.animation_action_client.send_goal(goal)
        self.animation_started = True

    def animation_finished(self, blackboard):
        state = blackboard.animation_action_client.get_state()
        return state == 3 

class PlayAnimationStandUp(AbstractPlayAnimation):

    def chose_animation(self, blackboard):
        # publish that we are getting up
        # blackboard.publish_state(STATE_GETTING_UP)
        side = blackboard.get_fallen_side()
        if side == blackboard.fall_checker.FRONT:
            rospy.loginfo("PLAYING STAND UP FRONT ANIMATION")
            return blackboard.stand_up_front_animation
        if side == blackboard.fall_checker.BACK:
            rospy.loginfo("PLAYING STAND UP BACK ANIMATION")
            return blackboard.stand_up_back_animation
        if side == blackboard.fall_checker.SIDE:
            rospy.loginfo("PLAYING STAND UP SIDE ANIMATION")
            return blackboard.stand_up_side_animation


class PlayAnimationFalling(AbstractPlayAnimation):

    def chose_animation(self, blackboard):
        # blackboard.publish_state(Fallen)
        side = blackboard.robot_falling_direction()
        if side == blackboard.fall_checker.FRONT:
            rospy.loginfo("PLAYING FALLING FRONT ANIMATION")
            return blackboard.falling_animation_front
        if side == blackboard.fall_checker.BACK:
            rospy.loginfo("PLAYING FALLING BACK ANIMATION")
            return blackboard.falling_animation_back
        if side == blackboard.fall_checker.LEFT:
            rospy.loginfo("PLAYING FALLING LEFT ANIMATION")
            return blackboard.falling_animation_left
        if side == blackboard.fall_checker.RIGHT:
            rospy.loginfo("PLAYING FALLING RIGHT ANIMATION")
            return blackboard.falling_animation_right

class PlayAnimationPenalty(AbstractPlayAnimation):

    def chose_animation(self, blackboard):
        return blackboard.penalty_animation


class PlayAnimationWalkready(AbstractPlayAnimation):

    def chose_animation(self, blackboard):
        return blackboard.walkready_animation

class PlayAnimationSitDown(AbstractPlayAnimation):

    def chose_animation(self, blackboard):
        return blackboard.sit_down_animation

class PlayAnimationMotorOff(AbstractPlayAnimation):

    def chose_animation(self, blackboard):
        return blackboard.motor_off_animation