# -*- coding:utf-8 -*-
"""
FilteredVisionInfoCapsule
^^^^^^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: sheepy <sheepy@informatik.uni-hamburg.de>

History:
* 5/9/14: Created (sheepy)

"""
from math import sqrt
import math

from bitbots.modules.keys import DATA_KEY_GOAL_MODEL, DATA_KEY_GOAL_INFO, DATA_KEY_BALL_SPEED, \
    DATA_KEY_BASELINE_INTERSECTION_DISTANCE, \
    DATA_KEY_BASELINE_INTERSECTION_TIME, DATA_KEY_BALL_INFO_SIMPLE_FILTERED, \
    DATA_KEY_GOAL_INFO_FILTERED


class FilteredVisionCapsule(object):
    def __init__(self, data):
        self.data = data

    def get_ball_speed(self):
        return self.data.get(DATA_KEY_BALL_SPEED, 0)

    def get_simple_filtered_ball_info(self):
        """
        Returns a namedtuple

        namedtuple("filteredBallInfo", ("u", "v", "distance", "angle", "uestimate", "vestimate" "time"))

            u,v                     ->  vertikale/horizontale Entfernung zum Ball
            distance                ->  Luftlinienentfernung zum Ball
            angle                   ->  Winkel, unter dem der Ball gesehen wird
            uestimate, vestimate    ->  vorhergesagte Werte wo der Ball landen wird
            time                    ->  Zeitpunkt, zu dem die Ballinfos gespeichert wurden
        """
        return self.data[DATA_KEY_BALL_INFO_SIMPLE_FILTERED]

    # Updated to SimpleFiltered
    def get_uv_estimate(self):
        # return self.data[DATA_KEY_BALL_INFO_FILTERED]["uvprediction"]
        return self.data[DATA_KEY_BALL_INFO_SIMPLE_FILTERED].uestimate, self.data[
            DATA_KEY_BALL_INFO_SIMPLE_FILTERED].vestimate

    def get_uv_distance_estimate(self):
        return math.sqrt((self.get_uv_estimate()[0] ** 2) + (self.get_uv_estimate()[1] ** 2))

    def get_baseline_intersection_distance(self):
        return self.data[DATA_KEY_BASELINE_INTERSECTION_DISTANCE]

    def get_baseline_intersection_time(self):
        return self.data[DATA_KEY_BASELINE_INTERSECTION_TIME]

    def get_local_goal_model_goals(self):
        """
        :return List of tupels with u and v data for each Goal. Eg:[(u,v),(u,v)]
        """
        return self.data[DATA_KEY_GOAL_MODEL].get_goals()

    def get_local_goal_model_ball(self):
        """
        :rtype tuple
        :return (u,v)
        """
        return self.data[DATA_KEY_GOAL_MODEL].get_ball_position()

    def get_local_goal_model_ball_distance(self):
        u, v = self.data[DATA_KEY_GOAL_MODEL].get_ball_position()
        return math.sqrt(u**2 + v**2)

    def get_local_goal_filtered(self):
        """
        Returns (custom dbscan) filtered data of goal information.

        u_center/v_center  ->  vertical/horizontal distance to the center of the goal
        u_post1/v_post1    ->  vertical/horizontal distance to goal post one
        u_post2/v_post2    ->  vertical/horizontal distance to goal post two
        time               ->  point in time of the current data set (rospy.get_time() or it wont work in simulator)

        :rtype :  namedtuple
        :return : (u_center, v_center, u_post1, v_post1, u_post2, v_post2, time)
        """
        return self.data[DATA_KEY_GOAL_INFO_FILTERED]

    def get_local_goal_model_own_goal(self):
        return self.data[DATA_KEY_GOAL_MODEL].get_own_goal()

    def get_local_goal_model_own_goal_posts(self):
        """
        :return:[(u,v), (u,v)] of the own Goalpost
        """
        return self.data[DATA_KEY_GOAL_MODEL].get_own_goal_posts()

    def get_local_goal_model_own_goal_distance(self):
        return self.data[DATA_KEY_GOAL_MODEL].get_own_goal_distance()

    def get_local_goal_model_defender_point(self):
        return self.data[DATA_KEY_GOAL_MODEL].get_own_goal_defender_point()

    def get_local_goal_model_opp_goal(self):
        return self.data[DATA_KEY_GOAL_MODEL].get_opp_goal()

    def get_local_goal_model_opp_goal_posts(self):
        """
        :return:[(u,v), (u,v)] of the opp Goalpost
        """
        return self.data[DATA_KEY_GOAL_MODEL].get_opp_goal_posts()

    def get_local_goal_model_posts_uv(self):
        return self.data[DATA_KEY_GOAL_MODEL].get_goal_posts()

    def switch_local_goal_model_orientation(self):
        self.data[DATA_KEY_GOAL_MODEL].switch_local_goal_model_orientation()

    def get_local_goal_model(self):
        return self.data[DATA_KEY_GOAL_MODEL]

    def get_complete_goal_seen(self):
        """
        Is true when the vision sees two goal posts,
        that have a distance to each other that is possible a real goal.
        There is no information which goal this is
        """

        # two posts seen
        # Is true when the vision sees two goal posts, that have a distance to each other that is possible a real goal.
        # There is no information which goal this is
        # zwei pfosten gehsen
        if len(self.data.get(DATA_KEY_GOAL_INFO, [])) > 1:
            post1 = self.data[DATA_KEY_GOAL_INFO][0]
            post2 = self.data[DATA_KEY_GOAL_INFO][1]
            distance = sqrt((post1.u - post2.u) ** 2 + (post1.v - post2.v) ** 2)
            # debug_m(4, "Distance_between_goalposts", distance)
            if 100 < distance < 3000:  # todo config

                return True
            else:
                return False

    def get_center_of_seen_goal(self):
        """
        :return Center of the seen goal if a goal was recognised. Else retuns None

        """
        if self.get_complete_goal_seen():
            post1 = self.data[DATA_KEY_GOAL_INFO][0]
            post2 = self.data[DATA_KEY_GOAL_INFO][1]
            p1_to_p2 = (post2.u - post1.u, post2.v - post1.v)
            center = (post1.u + (0.5 * p1_to_p2[0]), post1.v + (0.5 * p1_to_p2[1]))
            return center
        else:
            return None
