"""
AnimationCapsule
^^^^^^^^^^^^^^^^
"""
import actionlib
import rospy
from humanoid_league_msgs.msg import PlayAnimationGoal, PlayAnimationAction


class AnimationCapsule:
    def __init__(self):
        self.active = False
        self.animation_client = actionlib.SimpleActionClient('animation', PlayAnimationAction)

    def play_animation(self, animation):
        """
        plays the animation "ani" and sets the flag "BusyAnimation"

        :param animation: name of the animation which shall be played
        """
        if self.active:
            return False

        if animation is None or animation == "":
            rospy.logwarn("Tried to play an animation with an empty name!")
            return False
        first_try = self.animation_client.wait_for_server(
            rospy.Duration(rospy.get_param("hcm/anim_server_wait_time", 10)))
        if not first_try:
            rospy.logerr(
                "Animation Action Server not running! Motion can not work without animation action server. "
                "Will now wait until server is accessible!")
            self.animation_client.wait_for_server()
            rospy.logwarn("Animation server now running, hcm will go on.")
        goal = PlayAnimationGoal()
        goal.animation = animation
        goal.hcm = False  # the animation is from the hcm
        self.animation_client.send_goal(goal, done_cb=self.cb_unset_is_busy)
        self.active = True

    def cb_unset_is_busy(self, _p1, _p2):
        self.active = False

    def is_animation_busy(self):
        """Checks if an animation is currently played"""
        return self.active
