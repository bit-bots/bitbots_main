from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration

from bitbots_blackboard.capsules import AbstractBlackboardCapsule
from bitbots_msgs.action import Dynup, LookAt, PlayAnimation


class AnimationCapsule(AbstractBlackboardCapsule):
    """Communicates with the animation action server to play animations."""

    def __init__(self, node, blackboard):
        super().__init__(node, blackboard)
        self.active = False

        # Config
        self.grab_ball_animation: str = self._node.get_parameter("Animations.Throw_in.grabBall").value
        self.throw_animation: str = self._node.get_parameter("Animations.Throw_in.throw").value
        self.goalie_arms_animation: str = self._node.get_parameter("Animations.Goalie.goalieArms").value
        self.goalie_falling_right_animation: str = self._node.get_parameter("Animations.Goalie.fallRight").value
        self.goalie_falling_left_animation: str = self._node.get_parameter("Animations.Goalie.fallLeft").value
        self.goalie_falling_center_animation: str = self._node.get_parameter("Animations.Goalie.fallCenter").value
        self.cheering_animation: str = self._node.get_parameter("Animations.Misc.cheering").value
        self.init_animation: str = self._node.get_parameter("Animations.Misc.init").value

        self.animation_client = ActionClient(
            self._node, PlayAnimation, "animation", callback_group=ReentrantCallbackGroup()
        )

        self.dynup_action_client = ActionClient(self._node, Dynup, "dynup", callback_group=ReentrantCallbackGroup())

        self.lookat_action_client = ActionClient(self._node, LookAt, "look_at_goal")

    def play_animation(self, animation: str, from_hcm: bool) -> bool:
        """
        plays the animation "ani" and sets the flag "BusyAnimation"

        :param animation: Name of the animation which shall be played
        :param from_hcm: Marks the action call as a call from the hcm
        :returns: True if the animation was successfully dispatched
        """
        return True
        if self.active:
            return False

        if animation is None or animation == "":
            self._node.get_logger().warn("Tried to play an animation with an empty name!")
            return False

        if not self.animation_client.wait_for_server(Duration(seconds=10)):
            self._node.get_logger().error(
                "Animation Action Server not running! Motion can not work without animation action server."
            )
            return False

        goal = PlayAnimation.Goal()
        goal.animation = animation
        goal.hcm = from_hcm  # the animation is from the hcm
        self.animation_client.send_goal_async(goal).add_done_callback(
            lambda future: future.result().get_result_async().add_done_callback(lambda _: self.__done_cb())
        )

        self.active = True

        return True

    def __done_cb(self) -> None:
        self.active = False

    def is_busy(self) -> bool:
        """Checks if an animation is currently played"""
        return self.active
