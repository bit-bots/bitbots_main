"""
WorldModellCapsule
^^^^^^^^^^^^^^^^^^

Provides informations about the world model.

"""
import math

import rospy
import tf2_ros as tf2
from tf2_geometry_msgs import PointStamped
from tf.transformations import euler_from_quaternion
from humanoid_league_msgs.msg import Position2D, ObstaclesRelative, GoalRelative, BallRelative


class WorldModelCapsule:
    def __init__(self, field_length, field_width):
        self.position = Position2D()
        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(30))
        self.tf_listener = tf2.TransformListener(self.tf_buffer)
        self.ball = PointStamped()  # The ball in the base footprint frame
        self.goal = GoalRelative()
        self.goal_odom = GoalRelative()
        self.goal_odom.header.stamp = rospy.Time.now()
        self.goal_odom.header.frame_id = 'odom'
        self.obstacles = ObstaclesRelative()
        self.my_data = dict()
        self.counter = 0
        self.ball_seen_time = rospy.Time(0)
        self.goal_seen_time = rospy.Time(0)
        self.ball_seen = False
        self.field_length = field_length
        self.field_width = field_width

    def get_current_position(self):
        return self.position.pose.x, self.position.pose.y, self.position.pose.theta

    ############
    ### Ball ###
    ############

    def ball_last_seen(self):
        return self.ball_seen_time

    def get_ball_position_xy(self):
        """Calculate the absolute position of the ball"""
        u, v = self.get_ball_position_uv()
        return self.get_xy_from_uv(u, v)

    def get_ball_stamped(self):
        return self.ball

    def get_ball_position_uv(self):
        return self.ball.point.x, self.ball.point.y

    def get_ball_distance(self):
        u, v = self.get_ball_position_uv()
        return math.sqrt(u ** 2 + v ** 2)

    def get_ball_speed(self):
        raise NotImplementedError

    def ball_callback(self, ball):
        # type: (BallRelative) -> None
        if ball.confidence == 0:
            return

        # adding a minor delay to timestamp to ease transformations.
        ball.header.stamp = ball.header.stamp + rospy.Duration.from_sec(0.01)
        ball_buffer = PointStamped(ball.header, ball.ball_relative)
        if ball.header.frame_id != 'base_footprint':
            try:
                self.ball = self.tf_buffer.transform(ball_buffer, 'base_footprint', timeout=rospy.Duration(0.3))
                self.ball_seen_time = rospy.Time.now()
                self.ball_seen = True

            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                rospy.logwarn(e)
        else:
            self.ball = ball_buffer
            self.ball_seen_time = rospy.Time.now()
            self.ball_seen = True

    def forget_ball(self):
        """Forget that we saw a ball"""
        self.ball_seen_time = rospy.Time(0)
        self.ball = PointStamped()

    ###########
    # ## Goal #
    ###########

    def goal_last_seen(self):
        # We are currently not seeing any goal, we know where they are based
        # on the localisation. Therefore, any_goal_last_seen returns the time
        # from the stamp of the last position update
        return self.goal_seen_time

    def get_map_based_opp_goal_center_uv(self):
        x, y = self.get_map_based_opp_goal_center_xy()
        return self.get_uv_from_xy(x, y)

    def get_map_based_opp_goal_center_xy(self):
        return self.field_length / 2, 0

    def get_map_based_own_goal_center_uv(self):
        x, y = self.get_map_based_own_goal_center_xy()
        return self.get_uv_from_xy(x, y)

    def get_map_based_own_goal_center_xy(self):
        return -self.field_length / 2, 0

    def get_map_based_opp_goal_angle_from_ball(self):
        ball_x, ball_y = self.get_ball_position_xy()
        goal_x, goal_y = self.get_map_based_opp_goal_center_xy()
        return math.atan2(goal_y - ball_y, goal_x - ball_x)

    def get_map_based_opp_goal_distance(self):
        x, y = self.get_map_based_opp_goal_center_xy()
        return self.get_distance_to_xy(x, y)

    def get_map_based_opp_goal_left_post_uv(self):
        x, y = self.get_map_based_opp_goal_center_xy()
        return self.get_uv_from_xy(x, y - self.goal_width / 2)

    def get_map_based_opp_goal_right_post_uv(self):
        x, y = self.get_map_based_opp_goal_center_xy()
        return self.get_uv_from_xy(x, y + self.goal_width / 2)

    def get_detection_based_goal_position_uv(self):
        """
        returns the position of the goal relative to the robot.
        if only a single post is detected, the position of the post is returned.
        else, it is the point between the posts
        :return:
        """
        left = PointStamped(self.goal_odom.header, self.goal_odom.left_post)
        right = PointStamped(self.goal_odom.header, self.goal_odom.right_post)
        try:
            left_bfp = self.tf_buffer.transform(left, 'base_footprint', timeout=rospy.Duration(0.2)).point
            right_bfp = self.tf_buffer.transform(right, 'base_footprint', timeout=rospy.Duration(0.2)).point
        except (tf2.ExtrapolationException) as e:
            rospy.logwarn(e)
            try:
                # retrying with latest time stamp available because the time stamp of the goal_odom.header
                # seems to be to young and an extrapolation would be required.
                left.header.stamp = rospy.Time(0)
                right.header.stamp = rospy.Time(0)
                left_bfp = self.tf_buffer.transform(left, 'base_footprint', timeout=rospy.Duration(0.2)).point
                right_bfp = self.tf_buffer.transform(right, 'base_footprint', timeout=rospy.Duration(0.2)).point
            except (tf2.ExtrapolationException) as e:
                rospy.logwarn(e)
                rospy.logerr('Severe transformation problem concerning the goal!')
                return None

        return (left_bfp.x + right_bfp.x / 2.0), \
               (left_bfp.y + right_bfp.y / 2.0)



    def goal_parts_callback(self, msg):
        # type: (GoalPartsRelative) -> None
        goal_parts = msg
        # todo: transform to base_footprint too!
        # adding a minor delay to timestamp to ease transformations.
        goal_parts.header.stamp = goal_parts.header.stamp + rospy.Duration.from_sec(0.01)

        # Tuple(First Post, Second Post, Distance)
        goal_combination = (-1,-1,-1)
        # Enumerate all goalpost combinations, this also combines each post with itself,
        # to get the special case that only one post was detected and the maximum distance is 0.
        for first_post_id, first_post in enumerate(goal_parts.posts):
            for second_post_id, second_post in enumerate(goal_parts.posts):
                # Get the minimal angular difference between the two posts
                angular_distance = abs((math.atan2(first_post.foot_point.x, first_post.foot_point.y) - math.atan2(second_post.foot_point.x, second_post.foot_point.y) + math.pi) % (2*math.pi) - math.pi)
                # Set a new pair of posts if the distance is bigger than the previous ones
                if angular_distance > goal_combination[2]:
                    goal_combination = (first_post_id, second_post_id, angular_distance)
        # Catch the case, that no posts are detected
        if goal_combination[2] == -1:
            return
        # Define right and left post
        first_post = goal_parts.posts[goal_combination[0]]
        second_post = goal_parts.posts[goal_combination[1]]
        if math.atan2(first_post.foot_point.y, first_post.foot_point.x) > \
                math.atan2(first_post.foot_point.y, first_post.foot_point.x):
            left_post = first_post
            right_post = second_post
        else:
            left_post = second_post
            right_post = first_post

        goal_left_buffer = PointStamped(goal_parts.header, left_post.foot_point)
        goal_right_buffer = PointStamped(goal_parts.header, right_post.foot_point)

        self.goal.header = goal_parts.header
        self.goal.left_post = goal_left_buffer
        self.goal.right_post = goal_right_buffer

        self.goal_odom.header = goal_parts.header
        if goal_left_buffer.header.frame_id != 'odom':
            try:
                self.goal_odom.left_post = self.tf_buffer.transform(goal_left_buffer, 'odom', timeout=rospy.Duration(0.2)).point
                self.goal_odom.right_post = self.tf_buffer.transform(goal_right_buffer, 'odom', timeout=rospy.Duration(0.2)).point
                self.goal_odom.header.frame_id = 'odom'
                self.goal_seen_time = rospy.Time.now()
            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                rospy.logwarn(e)
        else:
            self.goal_odom.left_post = goal_left_buffer.point
            self.goal_odom.right_post = goal_right_buffer.point
            self.goal_seen_time = rospy.Time.now()

    #############
    # ## Common #
    #############

    def get_uv_from_xy(self, x, y):
        """ Returns the relativ positions of the robot to this absolute position"""
        current_position = self.get_current_position()
        x2 = x - current_position[0]
        y2 = y - current_position[1]
        theta = -1 * current_position[2]
        u = math.cos(theta) * x2 + math.sin(theta) * y2
        v = math.cos(theta) * y2 - math.sin(theta) * x2
        return u, v

    def get_xy_from_uv(self, u, v):
        """ Returns the absolute position from the given relative position to the robot"""
        pos_x, pos_y, theta = self.get_current_position()
        angle = math.atan2(v, u) + theta
        hypotenuse = math.sqrt(u ** 2 + v ** 2)
        return pos_x + math.sin(angle) * hypotenuse, pos_y + math.cos(angle) * hypotenuse

    def get_distance_to_xy(self, x, y):
        """ Returns distance from robot to given position """
        u, v = self.get_uv_from_xy(x, y)
        dist = math.sqrt(u ** 2 + v ** 2)
        return dist

    def position_callback(self, pos):
        # Convert PositionWithCovarianceStamped to Position2D
        position2d = Position2D()
        position2d.header = pos.header
        position2d.pose.x = pos.pose.pose.position.x
        position2d.pose.y = pos.pose.pose.position.y
        rotation = pos.pose.pose.orientation
        position2d.pose.theta = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])[2]
        self.position = position2d
