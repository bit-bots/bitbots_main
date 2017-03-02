"""
AnimationCapsule
^^^^^^^^^^^^^^^^
"""
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from humanoid_league_msgs.msg import PlayAnimationGoal


class AnimationCapsule:
    def __init__(self):
        self.server = None  # type: actionlib.SimpleActionClient
        self.an_config = rospy.get_param("animations", None)
        if not self.an_config:
            rospy.logerr("Animations not found")

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
