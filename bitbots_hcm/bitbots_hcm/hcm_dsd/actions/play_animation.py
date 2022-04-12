import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
import humanoid_league_msgs.msg
import bitbots_msgs.msg
from actionlib_msgs.msg import GoalStatus
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from time import sleep

from humanoid_league_msgs.msg import RobotControlState


class AbstractPlayAnimation(AbstractActionElement):
    """
    Abstract class to create actions for playing animations
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters=None)
        self.first_perform = True

    def perform(self, reevaluate=False):
        # we never want to leave the action when we play an animation
        self.do_not_reevaluate()

        if self.first_perform:
            # get the animation that should be played
            # defined by implementations of this abstract class
            anim = self.chose_animation()

            # try to start animation
            sucess = self.start_animation(anim)
            # if we fail, we need to abort this action
            if not sucess:
                self.get_logger().error("Could not start animation. Will abort play animation action!")
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

        self.get_logger().info("Playing animation " + anim)
        if anim is None or anim == "":
            self.get_logger().warn("Tried to play an animation with an empty name!")
            return False
        first_try = self.blackboard.animation_action_client.wait_for_server(
            Duration(seconds=self.get_parameter('"hcm/anim_server_wait_time"').get_parameter_value().double_value)
        if not first_try:
            server_running = False
            while not server_running and not self.blackboard.shut_down_request and rclpy.ok():
                self.blackboard.node.logerr_throttle(5.0,
                                      "Animation Action Server not running! Motion can not work without animation action server. "
                                      "Will now wait until server is accessible!")
                server_running = self.blackboard.animation_action_client.wait_for_server(Duration(seconds=1))
            if server_running:
                self.get_logger().warn("Animation server now running, hcm will go on.")
            else:
                self.get_logger().warn("Animation server did not start.")
                return False
        goal = humanoid_league_msgs.msg.PlayAnimationGoal()
        goal.animation = anim
        goal.hcm = True  # the animation is from the hcm
        self.blackboard.animation_action_client.send_goal_async(goal)
        return True

    def animation_finished(self):
        state = self.blackboard.animation_action_client.get_state()
        return state in [GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]


class PlayAnimationStandUpFront(AbstractPlayAnimation):
    def chose_animation(self):
        self.blackboard.current_state = RobotControlState.GETTING_UP
        self.get_logger().info("PLAYING STAND UP FRONT ANIMATION")
        return self.blackboard.stand_up_front_animation


class PlayAnimationStandUpBack(AbstractPlayAnimation):
    def chose_animation(self):
        self.blackboard.current_state = RobotControlState.GETTING_UP
        self.get_logger().info("PLAYING STAND UP BACK ANIMATION")
        return self.blackboard.stand_up_back_animation


class PlayAnimationStandUpLeft(AbstractPlayAnimation):
    def chose_animation(self):
        self.blackboard.current_state = RobotControlState.GETTING_UP
        self.get_logger().info("PLAYING STAND UP LEFT ANIMATION")
        return self.blackboard.stand_up_left_animation


class PlayAnimationStandUpRight(AbstractPlayAnimation):
    def chose_animation(self):
        self.blackboard.current_state = RobotControlState.GETTING_UP
        self.get_logger().info("PLAYING STAND UP RIGHT ANIMATION")
        return self.blackboard.stand_up_right_animation


class PlayAnimationFallingLeft(AbstractPlayAnimation):
    def chose_animation(self):
        self.get_logger().info("PLAYING FALLING LEFT ANIMATION")
        return self.blackboard.falling_animation_left


class PlayAnimationFallingRight(AbstractPlayAnimation):
    def chose_animation(self):
        self.get_logger().info("PLAYING FALLING RIGHT ANIMATION")
        return self.blackboard.falling_animation_right


class PlayAnimationFallingFront(AbstractPlayAnimation):
    def chose_animation(self):
        self.get_logger().info("PLAYING FALLING FRONT ANIMATION")
        return self.blackboard.falling_animation_front


class PlayAnimationFallingBack(AbstractPlayAnimation):
    def chose_animation(self):
        self.get_logger().info("PLAYING FALLING BACK ANIMATION")
        return self.blackboard.falling_animation_back


class PlayAnimationStopped(AbstractPlayAnimation):
    def chose_animation(self):
        return self.blackboard.stop_animation


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
        super().__init__(blackboard, dsd, parameters=None)
        self.direction = parameters.get('direction')
        self.first_perform = True

        # A parameter 'initial' is passed when dynup is called during the startup phase,
        # in this case we do not want to set the state to GETTING_UP.
        initial = parameters.get('initial', False)
        if not initial:
            self.blackboard.current_state = RobotControlState.GETTING_UP

    def perform(self, reevaluate=False):
        # deactivate falling since it will be wrongly detected
        self.do_not_reevaluate()
        if self.first_perform:
            # get the animation that should be played
            # defined by implementations of this abstract class

            # try to start animation
            success = self.start_animation()
            # if we fail, we need to abort this action
            if not success:
                self.get_logger().error("Could not start animation. Will abort play animation action!")
                return self.pop()

            self.first_perform = False
            return

        if self.animation_finished():
            # we are finished playing this animation
            return self.pop()

    def start_animation(self):
        """
        This will NOT wait by itself. You have to check
        animation_finished()
        by yourself.
        :return:
        """

        first_try = self.blackboard.dynup_action_client.wait_for_server(
            Duration(seconds=self.get_parameter('"hcm/anim_server_wait_time"').get_parameter_value().double_value
        if not first_try:
            server_running = False
            while not server_running and not self.blackboard.shut_down_request and rclpy.ok():
                self.blackboard.node.logerr_throttle(5.0,
                                      "Dynup Action Server not running! Dynup cannot work without dynup server!"
                                      "Will now wait until server is accessible!")
                server_running = self.blackboard.dynup_action_client.wait_for_server(Duration(seconds=1))
            if server_running:
                self.get_logger().warn("Dynup server now running, hcm will go on.")
            else:
                self.get_logger().warn("Dynup server did not start.")
                return False
        goal = bitbots_msgs.msg.DynUpGoal()
        goal.direction = self.direction
        self.blackboard.dynup_action_client.send_goal_async(goal)
        return True

    def animation_finished(self):
        state = self.blackboard.dynup_action_client.get_state()
        return state in [GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]
