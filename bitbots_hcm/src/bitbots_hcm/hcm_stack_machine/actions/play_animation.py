import rospy 
import humanoid_league_msgs.msg
from bitbots_stackmachine.abstract_action_element import AbstractActionElement
from bitbots_hcm.hcm_stack_machine.hcm_connector import HcmConnector, STATE_GETTING_UP


class AbstractPlayAnimation(AbstractActionElement):
    """
    Abstract class to create actions for playing animations
    """

    def __init__(self, connector, _):
        super(AbstractPlayAnimation, self).__init__(connector)
        self.first_perform = True

    def perform(self, connector, reevaluate=False):        
        # we never want to leave the action when we play an animation
        # deactivate the reevaluate
        self.do_not_reevaluate()

        if self.first_perform:
            self.first_perform = False

            # get the animation that should be played
            # defined by implementations of this abstract class
            anim = self.chose_animation(connector)
            
            #start animation
            self.start_animation(connector, anim)
            return
        
        if self.animation_finished(connector):
            # we are finished playing this animation
            return self.pop()

    def chose_animation(self, connector):
        # this is what has to be implemented returning the animation to play
        raise NotImplementedError

    def chose_state_to_publish(self, connector):
        raise NotImplementedError

    def start_animation(self, connector, anim):
        """
        This will NOT wait by itself. You have to check
        animation_finished()
        by yourself.
        :param anim: animation to play
        :return:
        """
        connector.hcm_animation_playing = False  # will be set true when the hcm receives keyframe callback
        connector.hcm_animation_finished = False

        rospy.loginfo("Playing animation " + anim)
        if anim is None or anim == "":
            rospy.logwarn("Tried to play an animation with an empty name!")
            return False
        first_try = connector.animation_action_client.wait_for_server(
            rospy.Duration(rospy.get_param("hcm/anim_server_wait_time", 10)))
        if not first_try:
            rospy.logerr(
                "Animation Action Server not running! Motion can not work without animation action server. "
                "Will now wait until server is accessible!")
            connector.animation_action_client.wait_for_server()
            rospy.logwarn("Animation server now running, hcm will go on.")
        goal = humanoid_league_msgs.msg.PlayAnimationGoal()
        goal.animation = anim
        goal.hcm = True  # the animation is from the hcm
        connector.animation_action_client.send_goal(goal)
        self.animation_started = True

    def animation_finished(self, connector):
        return connector.hcm_animation_finished

class PlayAnimationStandUp(AbstractPlayAnimation):

    def chose_animation(self, connector):
        # publish that we are getting up
        connector.publish_state(STATE_GETTING_UP)

        side = connector.hcm.get_fallen_side()
        if side == "FRONT":
            return "StandUpFront"
        elif side == "BACK":
            return "StandUpBack"
        else:
            rospy.logerr("Fallen side " + side + " not known!")
            return 


class PlayAnimationFalling(AbstractPlayAnimation):

    def chose_animation(self, connector):
        #TODO implement
        pass

class PlayAnimationPenalty(AbstractPlayAnimation):

    def chose_animation(self, connector):
        return connector.penalty_animation        


class PlayAnimationWalkready(AbstractPlayAnimation):

    def chose_animation(self, connector):
        return connector.walkready_animation

class PlayAnimationSitDown(AbstractPlayAnimation):

    def chose_animation(self, connector):
        return connector.sit_down_animation      

class PlayAnimationMotorOff(AbstractPlayAnimation):

    def chose_animation(self, connector):
        return connector.motor_off_animation 