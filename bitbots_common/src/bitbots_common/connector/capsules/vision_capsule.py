import time
from typing import List

from humanoid_league_msgs.msg import ObstacleRelative, BallRelative, ObstaclesRelative, GoalRelative
import rospy

class VisionCapsule:
    def __init__(self):
        self.ball = BallRelative()
        self.goal = GoalRelative()
        self.obstacles = ObstaclesRelative()
        self.my_data = dict()

    ############
    # ## Ball ##
    ############

    def ball_seen(self):
        return rospy.get_time() - self.ball_last_seen() < 0.5

    def ball_last_seen(self):
        return self.my_data.get("BallLastSeen", -999)

    def get_ball_relative(self):
        return self.ball.ball_relative.x, self.ball.ball_relative.y

    def get_ball_distance(self):
        x, y = self.get_ball_relative()
        return (x**2 + y**2)**0.5

    ############
    # ## Goal ##
    ############

    def any_goal_seen(self):
        return rospy.get_time() - self.goal_last_seen() < 0.5

    def goal_last_seen(self):
        return self.my_data.get("GoalLastSeen", -999)

    def get_left_post_relative(self):
        return self.goal.left_post.x, self.goal.left_post.y

    def get_left_post_distance(self):
        x, y = self.get_left_post_relative()
        return (x**2 + y**2)**0.5

    def get_right_post_relative(self):
        if self.goal.right_post:
            return self.goal.right_post.x, self.goal.right_post.y
        else:
            return

    def get_right_post_distance(self):
        if self.goal.right_post:
            x, y = self.get_right_post_relative()
            return (x**2 + y**2)**0.5
        else:
            return

    def get_goal_relative(self):
        if self.goal.center_direction:
            return self.goal.center_direction.x, self.goal.center_direction.y
        else:
            # We have to calculate it
            if self.get_right_post_relative():
                x = (self.get_right_post_relative()[0] + self.get_left_post_relative()[0]) / 2
                y = (self.get_right_post_relative()[1] + self.get_left_post_relative()[1]) / 2
                return x, y
            else:
                return self.get_left_post_relative()

    def get_goal_distance(self):
        x, y = self.get_goal_relative()
        return (x**2 + y**2)**0.5

    #############
    # ## Other ##
    #############

    def get_last_seen(self, key):
        """ key: Returns the timestamp for the requested object when it was last seen """
        if key == "Ball":
            return self.my_data.get("BallLastSeen", 0)
        elif key == "Goal":
            return self.my_data.get("GoalLastSeen", 0)
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

    def ball_callback(self, ball: BallRelative):
        self.ball = ball
        self.my_data["BallLastSeen"] = rospy.get_time()

    def goal_callback(self, goal: GoalRelative):
        self.goal = goal
        self.my_data["GoalLastSeen"] = rospy.get_time()

    def obstacle_callback(self, obstacles: ObstaclesRelative):
        self.obstacles = obstacles
