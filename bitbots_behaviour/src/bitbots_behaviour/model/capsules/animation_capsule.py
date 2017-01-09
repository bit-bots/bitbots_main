"""
AnimationCapsule
^^^^^^^^^^^^^^^^
"""
import actionlib
from actionlib_msgs.msg import GoalStatus
from bitbots_animation.msg import PlayAnimationGoal


class AnimationCapsule:
    def __init__(self):
        self.server: actionlib.SimpleActionClient = None

    def play_animation(self, ani: str)->bool:
        """
        plays the animation "ani" and sets the flag "BusyAnimation"

        :param ani: name of the animation which shall be played
        """
        if self.server.get_goal_status() == GoalStatus.ACTIVE:
            # is currently busy, so can't start a new animation
            return False
        else:
            goal = PlayAnimationGoal()
            goal.animation = ani
            goal.force = False
            self.server.execute_cb(goal)
            return True

    def is_animation_busy(self):
        """
        Checks if an animation is currently played
        """
        return self.server.get_goal_status() == GoalStatus.ACTIVE
