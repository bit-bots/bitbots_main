import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class PostSeen(AbstractDecisionElement):
    """
    Decides whether a post is currently being seen.
    To be precise the decision checks if the post is currently getting located by our world-model.
    """

    def __init__(self, blackboard, dsd, parameters=None):
        super(PostSeen, self).__init__(blackboard, dsd, parameters)
        self.post_lost_time = self.blackboard.config['post_lost_time']

    def perform(self, reevaluate=False):
        """
        Check if the time we last saw the post is now.
        "Now"-Tolerance can be configured through post_lost_time.

        :param reevaluate: Has no effect
        """

        if rospy.get_time() - self.blackboard.world_model.goal_last_seen() < self.post_lost_time:
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        """
        True

        because we need to act immediately if the post_seen state changes
        """
        return True
