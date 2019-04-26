import rospy
from geometry_msgs.msg import PointStamped

from bitbots_head_behavior.actions.look_at import AbstractLookAt


class TrackPost(AbstractLookAt):
    """
    This action follows the seen post so that the camera always points towards it.
    We try to do this so that the post doesnt get lost as easily
    """

    def __init__(self, dsd, blackboard, parameters=None):
        super(TrackPost, self).__init__(dsd, blackboard, parameters)
        self.ball_tracking_min_pan_delta = self.blackboard.config['ball_tracking_min_pan_delta']
        self.ball_tracking_min_tilt_delta = self.blackboard.config['ball_tracking_min_tilt_delta']

    def perform(self, reevaluate=False):
        """
        Call look_at to look at the point which our world-model determines to be the post
        if both posts are seen, look in between

        :param reevaluate: No effect here
        """

        # Get last post position left or right, center if both are seen
        post_point = self.blackboard.world_model.get_detection_based_goal_position_uv()

        # Call internal look-at to turn head to this point (when necessary)
        self._look_at(post_point, self.ball_tracking_min_pan_delta, self.ball_tracking_min_tilt_delta)
