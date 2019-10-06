import rospy
from geometry_msgs.msg import PointStamped

from bitbots_head_behavior.actions.look_at import AbstractLookAt


class TrackBall(AbstractLookAt):
    """
    This action follows the seen ball so that it the camera always points towards it.
    We try to do this so that the ball doesnt get lost as easily
    """

    def __init__(self, dsd, blackboard, parameters=None):
        super(TrackBall, self).__init__(dsd, blackboard, parameters)
        self.ball_tracking_min_pan_delta = self.blackboard.config['ball_tracking_min_pan_delta']
        self.ball_tracking_min_tilt_delta = self.blackboard.config['ball_tracking_min_tilt_delta']

    def perform(self, reevaluate=False):
        """
        Call look_at to look at the point which our world-model determines to be the ball

        :param reevaluate: No effect here
        """

        # Get last ball position
        point = self.blackboard.world_model.get_ball_stamped()

        # Call internal look-at to turn head to this point (when necessary)
        self.look_at(point, self.ball_tracking_min_pan_delta, self.ball_tracking_min_tilt_delta)
