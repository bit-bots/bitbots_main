from rclpy.duration import Duration
import rclpy
import humanoid_league_msgs.msg
from actionlib_msgs.msg import GoalStatus
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
                self.blackboard.node.get_logger().error("Could not start animation. Will abort play animation action!")
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

        self.blackboard.node.get_logger().info("Playing animation " + anim)
        if anim is None or anim == "":
            self.blackboard.node.get_logger().warning("Tried to play an animation with an empty name!")
            return False
        first_try = self.blackboard.animation_action_client.wait_for_server(
            Duration(1))
        if not first_try:
            server_running = False
            while not server_running and not rclpy.ok():
                self.blackboard.node.get_logger().error_throttle(5.0,
                                      "Animation Action Server not running! Motion can not work without animation action server. "
                                      "Will now wait until server is accessible!")
                server_running = self.blackboard.animation_action_client.wait_for_server(Duration(1))
            if server_running:
                self.blackboard.node.get_logger().warning("Animation server now running, hcm will go on.")
            else:
                self.blackboard.node.get_logger().warning("Animation server did not start.")
                return False
        goal = humanoid_league_msgs.msg.PlayAnimationGoal()
        goal.animation = anim
        goal.hcm = True  # the animation is from the hcm
        self.blackboard.animation_action_client.send_goal(goal)
        return True

    def animation_finished(self):
        state = self.blackboard.animation_action_client.get_state()
        return state in [GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]


class PlayAnimationGoalieArms(AbstractPlayAnimation):
    def chose_animation(self):
        return self.blackboard.goalie_arms_animation


class PlayAnimationGoalieFallRight(AbstractPlayAnimation):
    def chose_animation(self):
        self.blackboard.node.get_logger().info("PLAYING GOALIE FALLING RIGHT ANIMATION")
        return self.blackboard.goalie_falling_right_animation


class PlayAnimationGoalieFallLeft(AbstractPlayAnimation):
    def chose_animation(self):
        self.blackboard.node.get_logger().info("PLAYING GOALIE FALLING LEFT ANIMATION")
        return self.blackboard.goalie_falling_left_animation

class PlayAnimationGoalieFallCenter(AbstractPlayAnimation):
    def chose_animation(self):
        self.blackboard.node.get_logger().info("PLAYING GOALIE FALLING CENTER ANIMATION")
        return self.blackboard.goalie_falling_center_animation


class PlayAnimationCheering(AbstractPlayAnimation):
    def chose_animation(self):
        self.blackboard.node.get_logger().info("PLAYING CHEERING ANIMATION")
        return self.blackboard.cheering_animation


class PlayAnimationInit(AbstractPlayAnimation):
    def chose_animation(self):
        return self.blackboard.init_animation
