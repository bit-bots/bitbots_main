import time

from humanoid_league_msgs.msg import ObstacleRelative, BallRelative, ObstaclesRelative, GoalRelative
import rospy
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point

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

    def get_ball_relative_msg(self):
        return self.ball.ball_relative

    def get_ball_relative_stamped(self):
        ball = PointStamped()
        ball.point = self.ball.ball_relative
        ball.header = self.ball.header
        return ball

    def get_ball_distance(self):
        x, y = self.get_ball_relative()
        return (x**2 + y**2)**0.5

    ############
    # ## Goal ##
    ############

    def any_goal_seen(self):
        return rospy.get_time() - self.any_goal_last_seen() < 0.5

    def any_goal_last_seen(self):
        return self.my_data.get("GoalLastSeen", -999)

    def get_goal_relative(self):
        if self.goal.center_direction:
            return self.goal.center_direction
        else:
            # We have to calculate it
            if self.get_right_post_relative():
                goal = Point()
                goal.x = (self.goal.right_post.x + self.goal.left_post.x) / 2
                goal.y = (self.goal.right_post.y + self.goal.left_post.y) / 2
                goal.z = (self.goal.right_post.z + self.goal.left_post.z) / 2
                return goal
            else:
                return self.goal.left_post

    def get_goal_relative_stamped(self):
        msg = PointStamped()
        msg.point = self.get_goal_relative()
        msg.header = self.goal.header()

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

    def get_obstacle_found(self):

        return len(self.obstacles.obstacles) > 0

    def get_obstacle_info(self):
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

    def ball_callback(self, ball):
        self.ball = ball
        self.my_data["BallLastSeen"] = rospy.get_time()

    def goal_callback(self, goal):
        self.goal = goal
        self.my_data["GoalLastSeen"] = rospy.get_time()

    def obstacle_callback(self, obstacles):
        self.obstacles = obstacles
