"""
WorldModellCapsule
^^^^^^^^^^^^^^^^^^

Provides informations about the world model.

"""
import math

from humanoid_league_msgs.msg import Position2D
from tf.transformations import euler_from_quaternion


class WorldModelCapsule:
    def __init__(self, ):
        self.position = Position2D()
        self.ball_position_u = None
        self.ball_position_v = None

    def get_current_position(self):
        return self.position.pose.x, self.position.pose.y, self.position.pose.theta

    def get_ball_position_xy(self):
        """Calculate the absolute position of the ball"""
        pos_x, pos_y, theta = self.get_current_position()
        ball_angle = math.atan2(self.ball_position_v, self.ball_position_u) + theta
        hypotenuse = math.sqrt(self.ball_position_u ** 2 + self.ball_position_v ** 2)
        return pos_x + math.sin(ball_angle) * hypotenuse, pos_y + math.cos(ball_angle) * hypotenuse

    def ball_relative_cb(self, msg):
        self.ball_position_u = msg.ball_relative.x
        self.ball_position_v = msg.ball_relative.y

    def get_ball_position_uv(self):
        return self.ball_position_u, self.ball_position_v

    def get_opp_goal_center_uv(self):
        raise NotImplementedError

    def get_opp_goal_center_xy(self):
        raise NotImplementedError

    def get_own_goal_center_uv(self):
        raise NotImplementedError

    def get_own_goal_center_xy(self):
        raise NotImplementedError

    def get_opp_goal_angle(self):
        raise NotImplementedError

    def get_opp_goal_distance(self):
        raise NotImplementedError

    def get_opp_goal_left_post_uv(self):
        raise NotImplementedError

    def get_opp_goal_right_post_uv(self):
        raise NotImplementedError

    def get_uv_from_xy(self, x, y):
        """ Returns the relativ positions of the robot to this absolute position"""
        current_position = self.get_current_position()
        x2 = x - current_position[0]
        y2 = y - current_position[1]
        theta = -1 * current_position[2]
        u = math.cos(theta) * x2 + math.sin(theta) * y2
        v = math.cos(theta) * y2 - math.sin(theta) * x2
        return u, v

    def get_distance_to_xy(self, x, y):
        """ Returns distance from robot to given position """
        u, v = self.get_uv_from_xy(x, y)
        dist = math.sqrt(u ** 2 + v ** 2)

        return dist

    def get_ballpos(self):
        raise NotImplementedError

    def position_callback(self, pos):
        # Convert PositionWithCovarianceStamped to Position2D
        position2d = Position2D()
        position2d.header = pos.header
        position2d.pose.x = pos.pose.pose.position.x
        position2d.pose.y = pos.pose.pose.position.y
        rotation = pos.pose.pose.orientation
        position2d.pose.theta = euler_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)[2]
        self.position = position2d
