"""
AnimationCapsule
^^^^^^^^^^^^^^^^
"""
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from humanoid_league_msgs.action import PlayAnimation


class AnimationCapsule:
    def __init__(self, node: Node):
        self.node = node
        self.active = False
        self.animation_client = ActionClient(
            node,
            PlayAnimation,
            'animation',
            callback_group=ReentrantCallbackGroup())

    def play_animation(self, animation: str, from_hcm: bool) -> bool:
        """
        plays the animation "ani" and sets the flag "BusyAnimation"

        :param animation: Name of the animation which shall be played
        :param from_hcm: Marks the action call as a call from the hcm
        :returns: True if the animation was succesfully depatched
        """
        if self.active:
            return False

        if animation is None or animation == "":
            self.node.get_logger().warn("Tried to play an animation with an empty name!")
            return False

        if not self.animation_client.wait_for_server(Duration(seconds=10)):
            self.node.get_logger().error(
                "Animation Action Server not running! Motion can not work without animation action server.")
            return False

        goal = PlayAnimation.Goal()
        goal.animation = animation
        goal.hcm = from_hcm  # the animation is from the hcm
        self.animation_client.send_goal_async(goal).add_done_callback(
            lambda future: future.result().get_result_async().add_done_callback(
                lambda _: self.__done_cb()))

        self.active = True

        return True

    def __done_cb(self) -> None:
        self.active = False

    def is_busy(self) -> bool:
        """Checks if an animation is currently played"""
        return self.active
