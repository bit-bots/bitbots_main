# -*- coding:utf-8 -*-
"""
RawVisionInfoCapsule
^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 5/7/14: Created (sheepy)

"""

from bitbots.modules.keys import DATA_KEY_GOAL_INFO, DATA_KEY_IS_NEW_FRAME, DATA_KEY_GOAL_FOUND, DATA_KEY_BALL_INFO, \
    DATA_KEY_BALL_FOUND, DATA_KEY_CURRENT_HORIZON_UV, DATA_KEY_CURRENT_HORIZON_ORIENTATION, \
    DATA_KEY_OBSTACLE_INFO, DATA_KEY_OBSTACLE_FOUND, DATA_KEY_ANY_GOALPOST_LAST_SEEN, \
    DATA_KEY_ANY_WHOLE_GOAL_LAST_SEEN, DATA_KEY_HORIZON_OBSTACLES


class RawVisionCapsule:
    def __init__(self, data):
        self.data = data

    ############
    # ## Ball ##
    ############

    def get_ball_info(self, key):
        """
        Gets information about the ball
        a and b are the positions on the image
        :type key: str
        :param key: u, v, a, b, radius, rating, or distance
        """
        assert key in ["u", "v", "a", "b", "x", "y", "radius", "rating",
                       "distance"]  # todo x,y wieder entfernen wenn legacy raus ist
        if self.get_last_seen("Ball") != 0:
            # Schaut ob schon Informationen zum Ball da sind
            return getattr(self.data[DATA_KEY_BALL_INFO], key)
        else:
            return None

    def get_ball_info_legacy_wrapper(self, key):  # todo HOTFIX, please change in vision x,y in a,b
        """
        Converts the a and b key in x and y
        :param key:
        :return:
        """
        if key == "a":
            key = "x"
        if key == "b":
            key = "y"
        return self.get_ball_info(key)

    def ball_seen(self):
        if self.data.get(DATA_KEY_BALL_FOUND, False):
            return True
        else:
            return False

    def ball_last_seen(self):
        return self.data.get("BallLastSeen", 0)

    ############
    # ## Goal ##
    ############

    def any_goal_seen(self):
        """
        :rtype bool
        :return: Has actually seen a (part of a) goal
        """
        return self.data.get(DATA_KEY_GOAL_FOUND, False)

    def any_whole_goal_seen(self):
        """
        :rtype bool
        :return: Has seen two posts wich match to a goal
        """
        return self.data.get(DATA_KEY_ANY_WHOLE_GOAL_LAST_SEEN, False)

    def any_goalpost_last_seen(self):
        """
        :rtype int
        :return: Timestamp of the last time a goalpost was seen
        """
        return self.data.get(DATA_KEY_ANY_GOALPOST_LAST_SEEN, 0)

    def own_goal_seen(self):
        raise NotImplementedError

    def enemy_goal_seen(self):
        raise NotImplementedError

    def get_goal_infos(self):
        """
        Holds a namedtuple (x, y, u, v) for each of the two goal posts (if found).
        """
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
        return self.data.get(DATA_KEY_CURRENT_HORIZON_UV, [None, None])

    def get_current_horizon_orientation(self):
        return self.data.get(DATA_KEY_CURRENT_HORIZON_ORIENTATION, 0)

    def get_horizon_obstacles(self):
        """

        :return: [(u,v),...]
        """
        return self.data.get(DATA_KEY_HORIZON_OBSTACLES, None)

    #############
    # ## Other ##
    #############

    def is_new_frame(self):
        return self.data[DATA_KEY_IS_NEW_FRAME]

    def get_last_seen(self, key):
        """ key: Returns the timestamp for the requested object when it was last seen """
        if key == "Ball":
            return self.data.get("BallLastSeen", 0)
        else:
            raise KeyError

    def get_obstacle_found(self):
        """
        """
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
        return self.data[DATA_KEY_OBSTACLE_INFO]
