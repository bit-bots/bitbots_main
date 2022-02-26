"""
AnimationCapsule
^^^^^^^^^^^^^^^^
"""
from rclpy.action import ActionClient
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from humanoid_league_msgs.msg import PlayAnimationGoal, PlayAnimationAction


class AnimationCapsule:
    def __init__(self, node: Node):
        self.node = node
        self.active = False
        self.animation_client = ActionClient(self, PlayAnimationAction, 'animation')

    def play_animation(self, animation):
        """
        plays the animation "ani" and sets the flag "BusyAnimation"

        :param animation: name of the animation which shall be played
        """
        if self.active:
            return False

        if animation is None or animation == "":
            self.get_logger().warn("Tried to play an animation with an empty name!")
            return False
        first_try = self.animation_client.wait_for_server(
            Duration(seconds=self.node.get_parameter("hcm/anim_server_wait_time").get_parameter_value().double_value))
        if not first_try:
            self.get_logger().error(
                "Animation Action Server not running! Motion can not work without animation action server. "
                "Will now wait until server is accessible!")
            self.animation_client.wait_for_server()
            self.get_logger().warn("Animation server now running, hcm will go on.")
        goal = PlayAnimationGoal()
        goal.animation = animation
        goal.hcm = False  # the animation is from the hcm
        self.animation_client.send_goal_async(goal, done_cb=self.cb_unset_is_busy)
        self.active = True

    def cb_unset_is_busy(self, _p1, _p2):
        self.active = False

    def is_animation_busy(self):
        """Checks if an animation is currently played"""
        return self.active
