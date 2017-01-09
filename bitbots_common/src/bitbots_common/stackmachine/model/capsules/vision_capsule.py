import time
from humanoid_league_msgs.msg import ObstacleRelative, BallRelative


class VisionCapsule:
    def __init__(self):
        self.ball = BallRelative()
        self.obstacle = ObstacleRelative()
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

    ############
    # ## Goal ##
    ############

    def any_goal_seen(self):
        """
        :rtype bool
        :return: Has actually seen a (part of a) goal
        """
        raise NotImplementedError
        return self.data.get(DATA_KEY_GOAL_FOUND, False)

    def any_whole_goal_seen(self):
        """
        :rtype bool
        :return: Has seen two posts wich match to a goal
        """
        raise NotImplementedError
        return self.data.get(DATA_KEY_ANY_WHOLE_GOAL_LAST_SEEN, False)

    def any_goalpost_last_seen(self):
        """
        :rtype int
        :return: Timestamp of the last time a goalpost was seen
        """
        raise NotImplementedError
        return self.data.get(DATA_KEY_ANY_GOALPOST_LAST_SEEN, 0)

    def own_goal_seen(self):
        raise NotImplementedError

    def enemy_goal_seen(self):
        raise NotImplementedError

    def get_goal_infos(self):
        """
        Holds a namedtuple (x, y, u, v) for each of the two goal posts (if found).
        """
        raise NotImplementedError
        return self.data.get(DATA_KEY_GOAL_INFO, None)

    def get_goal_post_count(self):
        goalinfo = self.get_goal_infos()
        if goalinfo is None:
            return 0
        else:
            return len(goalinfo.keys())

    ###############
    # ## Horizon ##
    ###############

    def get_current_horizon(self):
        raise NotImplementedError
        return self.data.get(DATA_KEY_CURRENT_HORIZON_UV, [None, None])

    def get_current_horizon_orientation(self):
        raise NotImplementedError
        return self.data.get(DATA_KEY_CURRENT_HORIZON_ORIENTATION, 0)

    def get_horizon_obstacles(self):
        """

        :return: [(u,v),...]
        """
        raise NotImplementedError
        return self.data.get(DATA_KEY_HORIZON_OBSTACLES, None)

    #############
    # ## Other ##
    #############

    def is_new_frame(self):
        raise NotImplementedError
        return self.data[DATA_KEY_IS_NEW_FRAME]

    def get_last_seen(self, key):
        """ key: Returns the timestamp for the requested object when it was last seen """
        if key == "Ball":
            return self.my_data.get("BallLastSeen", 0)
        else:
            raise KeyError

    def get_obstacle_found(self):
        """
        """
        raise NotImplementedError
        return self.data[DATA_KEY_OBSTACLE_FOUND]

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
        raise NotImplementedError
        return self.data[DATA_KEY_OBSTACLE_INFO]

    def ball_callback(self, ball: BallRelative):
        self.ball = ball
        self.my_data["BallLastSeen"] = time.time()

    def obstacle_callback(self, obstacle: ObstacleRelative):
        self.obstacle = obstacle

