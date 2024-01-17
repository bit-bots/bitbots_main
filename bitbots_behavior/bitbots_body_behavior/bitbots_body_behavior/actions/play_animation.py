from abc import ABC, abstractmethod

from dynamic_stack_decider.abstract_action_element import AbstractActionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class AbstractPlayAnimation(AbstractActionElement, ABC):
    """
    Abstract class to create actions for playing animations
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters=None)
        self.blackboard: BodyBlackboard

        self.first_perform = True
        self.finished = False
        self.from_hcm = parameters.get("from_hcm", False)

    def perform(self, reevaluate=False):
        # we never want to leave the action when we play an animation
        self.do_not_reevaluate()

        if self.first_perform:
            # get the animation that should be played
            # defined by implementations of this abstract class
            anim = self.get_animation_name()

            # try to start animation
            success = self.blackboard.animation.play_animation(anim, self.from_hcm)
            # if we fail, we need to abort this action
            if not success:
                self.blackboard.node.get_logger().error("Could not start animation. Will abort play animation action!")
                return self.pop()

            self.first_perform = False
            return

        if not self.blackboard.animation.is_busy():
            # we are finished playing this animation
            return self.pop()

    @abstractmethod
    def get_animation_name(self) -> str:
        # this is what has to be implemented returning the animation to play
        raise NotImplementedError


class PlayAnimationGoalieArms(AbstractPlayAnimation):
    def get_animation_name(self):
        return self.blackboard.animation.goalie_arms_animation


class PlayAnimationGoalieFallRight(AbstractPlayAnimation):
    def get_animation_name(self):
        self.blackboard.node.get_logger().info("PLAYING GOALIE FALLING RIGHT ANIMATION")
        return self.blackboard.animation.goalie_falling_right_animation


class PlayAnimationGoalieFallLeft(AbstractPlayAnimation):
    def get_animation_name(self):
        self.blackboard.node.get_logger().info("PLAYING GOALIE FALLING LEFT ANIMATION")
        return self.blackboard.animation.goalie_falling_left_animation


class PlayAnimationGoalieFallCenter(AbstractPlayAnimation):
    def get_animation_name(self):
        self.blackboard.node.get_logger().info("PLAYING GOALIE FALLING CENTER ANIMATION")
        return self.blackboard.animation.goalie_falling_center_animation


class PlayAnimationCheering(AbstractPlayAnimation):
    def get_animation_name(self):
        self.blackboard.node.get_logger().info("PLAYING CHEERING ANIMATION")
        return self.blackboard.animation.cheering_animation


class PlayAnimationInit(AbstractPlayAnimation):
    def get_animation_name(self):
        return self.blackboard.animation.init_animation


class PlayAnimationInitInSim(PlayAnimationInit):
    def perform(self, reevaluate=False):
        if self.blackboard.in_sim:
            return super().perform(reevaluate)
        else:
            return self.pop()
