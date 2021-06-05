import rospy
import humanoid_league_msgs.msg
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class AbstractPlayAnimation(AbstractActionElement):
    """
    Abstract class to create actions for playing animations
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(AbstractPlayAnimation, self).__init__(blackboard, dsd, parameters=None)

        self.first_perform = True

    def perform(self, reevaluate=False):
        # we never want to leave the action when we play an animation
        self.do_not_reevaluate()

        if self.first_perform:
            # get the animation that should be played
            # defined by implementations of this abstract class
            anim = self.chose_animation()

            # try to start animation
            success = self.start_animation(anim)
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
        return state >= 1


class PlayAnimationGoalieFallRight(AbstractPlayAnimation):
    def chose_animation(self):
        rospy.loginfo("PLAYING GOALIE FALLING RIGHT ANIMATION")
        return self.blackboard.goalie_falling_right_animation

class PlayAnimationGoalieFallLeft(AbstractPlayAnimation):
    def chose_animation(self):
        rospy.loginfo("PLAYING GOALIE FALLING LEFT ANIMATION")
        return self.blackboard.goalie_falling_left_animation