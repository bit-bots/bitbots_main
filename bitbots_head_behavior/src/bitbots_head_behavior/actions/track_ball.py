from geometry_msgs.msg import PointStamped

from bitbots_head_behavior.actions.look_at import AbstractLookAt


class TrackBall(AbstractLookAt):
    def __init__(self, dsd, blackboard, parameters=None):
        super(TrackBall, self).__init__(dsd, blackboard, parameters)

    def perform(self, reevaluate=False):
        # Construct target point from target direction
        point = PointStamped()
        point.header.frame_id = 'base_footprint'
        u, v = self.blackboard.world_model.get_ball_position_uv()
        point.point.x = u
        point.point.y = v
        point.point.z = 0

        # Call internal look-at to turn head to this point
        self._look_at(point)
        pass
