import time
from typing import List

from humanoid_league_msgs.msg import ObstacleRelative, BallRelative, ObstaclesRelative


class VisionCapsule:
    def __init__(self):
        self.ball = BallRelative()
        self.obstacles = ObstaclesRelative()
        self.my_data = dict()

    ############
    # ## Ball ##
    ############

    def ball_seen(self):
        return bool(self.ball.ball_relative)

    def ball_last_seen(self):
        return self.my_data.get("BallLastSeen", 0)

    def get_ball_relative(self):
        return self.ball.ball_relative.x, self.ball.ball_relative.y

    def get_ball_distance(self):
        x, y = self.get_ball_relative()
        return (x**2 + y**2)**0.5

    #############
    # ## Other ##
    #############

    def get_last_seen(self, key):
        """ key: Returns the timestamp for the requested object when it was last seen """
        if key == "Ball":
            return self.my_data.get("BallLastSeen", 0)
        else:
            raise KeyError

    def get_obstacle_found(self)->bool:

        return len(self.obstacles.obstacles) > 0

    def get_obstacle_info(self)->List[ObstacleRelative]:
        """
        Liefert ObstacleInfos

        u, v, x, y, h, w, color-key
        
        u: vertical distance
        v: horizontal distance
        x, y: x,y coordinates in the actual image
        h, w: height and width of the Obstacle
        color: 0 nothing, 1 magenta, 2 only magenta, 4 cyan, 8 only cyan, 16 only colored
        """

        return self.obstacles.obstacles

    def any_goal_seen(self):
        return False  # TODO get goal information

    def ball_callback(self, ball: BallRelative):
        self.ball = ball
        self.my_data["BallLastSeen"] = time.time()

    def obstacle_callback(self, obstacles: ObstaclesRelative):
        self.obstacles = obstacles
