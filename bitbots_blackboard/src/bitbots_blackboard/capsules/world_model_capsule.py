"""
WorldModelCapsule
^^^^^^^^^^^^^^^^^^

Provides information about the world model.
"""
import math
import ros_numpy
import numpy as np
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

import rospy
import tf2_ros as tf2
from std_msgs.msg import Header
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, TwistWithCovarianceStamped, TwistStamped
from tf.transformations import euler_from_quaternion
from humanoid_league_msgs.msg import PoseWithCertaintyArray, PoseWithCertainty


class GoalRelative:
    header = Header()
    left_post = Point()
    right_post = Point()

    def to_pose_with_certainty_array(self):
        p = PoseWithCertaintyArray()
        p.header = self.header
        l = PoseWithCertainty()
        l.pose.pose.position = self.left_post
        r = PoseWithCertainty()
        r.pose.pose.position = self.right_post
        p.poses = [l, r]
        return p


class WorldModelCapsule:
    def __init__(self):
        # This pose is not supposed to be used as robot pose. Just as precision measurement for the TF position.
        self.pose = PoseWithCovarianceStamped()
        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(30))
        self.tf_listener = tf2.TransformListener(self.tf_buffer)

        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.ball_frame = rospy.get_param('~ball_frame', 'ball')
        self.base_footprint_frame = rospy.get_param('~base_footprint_frame', 'base_footprint')

        self.ball = PointStamped()  # The ball in the base footprint frame
        self.ball_odom = PointStamped()  # The ball in the odom frame (when localization is not usable)
        self.ball_odom.header.stamp = rospy.Time(0)
        self.ball_odom.header.frame_id = self.odom_frame
        self.ball_map = PointStamped()  # The ball in the map frame (when localization is usable)
        self.ball_map.header.stamp = rospy.Time(0)
        self.ball_map.header.frame_id = self.map_frame
        self.ball_twist_map = None
        self.ball_twist_lost_time = rospy.Duration(rospy.get_param('behavior/body/ball_twist_lost_time', 2))
        self.ball_twist_precision_threshold = rospy.get_param('behavior/body/ball_twist_precision_threshold', None)

        self.goal = GoalRelative()  # The goal in the base footprint frame
        self.goal_odom = GoalRelative()
        self.goal_odom.header.stamp = rospy.Time.now()
        self.goal_odom.header.frame_id = self.odom_frame

        self.my_data = dict()
        self.counter = 0
        self.ball_seen_time = rospy.Time(0)
        self.goal_seen_time = rospy.Time(0)
        self.ball_seen = False
        self.field_length = rospy.get_param('field_length', None)
        self.field_width = rospy.get_param('field_width', None)
        self.goal_width = rospy.get_param('goal_width', None)

        self.use_localization = rospy.get_param('behavior/body/use_localization', None)

        self.pose_precision_threshold = rospy.get_param('behavior/body/pose_precision_threshold', None)
        self.pose_lost_time = rospy.get_param('behavior/body/pose_lost_time', None)

        # Publisher for visualization in RViZ
        self.ball_publisher = rospy.Publisher('debug/viz_ball', PointStamped, queue_size=1)
        self.goal_publisher = rospy.Publisher('debug/viz_goal', PoseWithCertaintyArray, queue_size=1)
        self.ball_twist_publisher = rospy.Publisher('debug/ball_twist', TwistStamped, queue_size=1)

        self.calc_gradient_map()

    ############
    ### Ball ###
    ############

    def ball_last_seen(self):
        return self.ball_seen_time

    def get_ball_position_xy(self):
        """Return the ball saved in the map or odom frame"""
        if self.use_localization and \
                self.localization_precision_in_threshold():
            ball = self.ball_map
        else:
            ball = self.ball_odom
        return ball.point.x, ball.point.y

    def get_ball_stamped(self):
        return self.ball

    def get_ball_position_uv(self):
        if self.use_localization and \
                self.localization_precision_in_threshold():
            ball = self.ball_map
        else:
            ball = self.ball_odom
        try:
            ball_bfp = self.tf_buffer.transform(ball, self.base_footprint_frame, timeout=rospy.Duration(0.2)).point
        except (tf2.ExtrapolationException) as e:
            rospy.logwarn(e)
            rospy.logerr('Severe transformation problem concerning the ball!')
            return None
        return ball_bfp.x, ball_bfp.y

    def get_ball_distance(self):
        u, v = self.get_ball_position_uv()
        return math.sqrt(u ** 2 + v ** 2)

    def get_ball_speed(self):
        raise NotImplementedError

    def balls_callback(self, msg: PoseWithCertaintyArray):
        if msg.poses:
            balls = sorted(msg.poses, reverse=True, key=lambda ball: ball.confidence)  # Sort all balls by confidence
            ball = balls[0]  # Ball with highest confidence

            if ball.confidence == 0:
                return

            ball_buffer = PointStamped(msg.header, ball.pose.pose.position)
            try:
                self.ball = self.tf_buffer.transform(ball_buffer, self.base_footprint_frame, timeout=rospy.Duration(0.3))
                self.ball_odom = self.tf_buffer.transform(ball_buffer, self.odom_frame, timeout=rospy.Duration(0.3))
                self.ball_map = self.tf_buffer.transform(ball_buffer, self.map_frame, timeout=rospy.Duration(0.3))
                # Set timestamps to zero to get the newest transform when this is transformed later
                self.ball_odom.header.stamp = rospy.Time(0)
                self.ball_map.header.stamp = rospy.Time(0)
                self.ball_seen_time = rospy.Time.now()
                self.ball_seen = True

            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                rospy.logwarn(e)

            self.ball_publisher.publish(self.ball)
        else:
            return

    def recent_ball_twist_available(self):
        if self.ball_twist_map is None:
            return False
        return rospy.Time.now() - self.ball_twist_map.header.stamp < self.ball_twist_lost_time

    def ball_twist_callback(self, msg: TwistWithCovarianceStamped):
        x_sdev = msg.twist.covariance[0]  # position 0,0 in a 6x6-matrix
        y_sdev = msg.twist.covariance[7]  # position 1,1 in a 6x6-matrix
        if x_sdev > self.ball_twist_precision_threshold['x_sdev'] or \
           y_sdev > self.ball_twist_precision_threshold['y_sdev']:
            return
        if msg.header.frame_id != self.map_frame:
            try:
                # point (0,0,0)
                point_a = PointStamped()
                point_a.header = msg.header
                # linear velocity vector
                point_b = PointStamped()
                point_b.header = msg.header
                point_b.point.x = msg.twist.twist.linear.x
                point_b.point.y = msg.twist.twist.linear.y
                point_b.point.z = msg.twist.twist.linear.z
                # transform start and endpoint of velocity vector
                point_a = self.tf_buffer.transform(point_a, self.map_frame, timeout=rospy.Duration(0.3))
                point_b = self.tf_buffer.transform(point_b, self.map_frame, timeout=rospy.Duration(0.3))
                # build new twist using transform vector
                self.ball_twist_map = TwistStamped(header=msg.header)
                self.ball_twist_map.header.frame_id = self.map_frame
                self.ball_twist_map.twist.linear.x = point_b.point.x - point_a.point.x
                self.ball_twist_map.twist.linear.y = point_b.point.y - point_a.point.y
                self.ball_twist_map.twist.linear.z = point_b.point.z - point_a.point.z
            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                rospy.logwarn(e)
        else:
            self.ball_twist_map = TwistStamped(header=msg.header, twist=msg.twist.twist)
        self.ball_twist_publisher.publish(self.ball_twist_map)

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

    def get_map_based_opp_goal_angle(self):
        x, y = self.get_map_based_opp_goal_center_uv()
        return math.atan2(y, x)

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
        left.header.stamp = rospy.Time(0)
        right.header.stamp = rospy.Time(0)
        try:
            left_bfp = self.tf_buffer.transform(left, self.base_footprint_frame, timeout=rospy.Duration(0.2)).point
            right_bfp = self.tf_buffer.transform(right, self.base_footprint_frame, timeout=rospy.Duration(0.2)).point
        except tf2.ExtrapolationException as e:
            rospy.logwarn(e)
            rospy.logerr('Severe transformation problem concerning the goal!')
            return None

        return (left_bfp.x + right_bfp.x / 2.0), \
               (left_bfp.y + right_bfp.y / 2.0)

    def goal_parts_callback(self, msg):
        # type: (GoalPartsRelative) -> None
        goal_parts = msg

    def goalposts_callback(self, goal_parts: PoseWithCertaintyArray):
        # todo: transform to base_footprint too!
        # adding a minor delay to timestamp to ease transformations.
        goal_parts.header.stamp = goal_parts.header.stamp + rospy.Duration.from_sec(0.01)

        # Tuple(First Post, Second Post, Distance)
        goal_combination = (-1, -1, -1)
        # Enumerate all goalpost combinations, this also combines each post with itself,
        # to get the special case that only one post was detected and the maximum distance is 0.
        for first_post_id, first_post in enumerate(goal_parts.poses):
            for second_post_id, second_post in enumerate(goal_parts.poses):
                # Get the minimal angular difference between the two posts
                first_post_pos = first_post.pose.pose.position
                second_post_pos = second_post.pose.pose.position
                angular_distance = abs((math.atan2(first_post_pos.x, first_post_pos.y) - math.atan2(
                    second_post_pos.x, second_post_pos.y) + math.pi) % (2*math.pi) - math.pi)
                # Set a new pair of posts if the distance is bigger than the previous ones
                if angular_distance > goal_combination[2]:
                    goal_combination = (first_post_id, second_post_id, angular_distance)
        # Catch the case, that no posts are detected
        if goal_combination[2] == -1:
            return
        # Define right and left post
        first_post = goal_parts.poses[goal_combination[0]].pose.pose.position
        second_post = goal_parts.poses[goal_combination[1]].pose.pose.position
        if math.atan2(first_post.y, first_post.x) > \
                math.atan2(first_post.y, first_post.x):
            left_post = first_post
            right_post = second_post
        else:
            left_post = second_post
            right_post = first_post

        self.goal.header = goal_parts.header
        self.goal.left_post = left_post
        self.goal.right_post = right_post

        self.goal_odom.header = goal_parts.header
        if goal_parts.header.frame_id != self.odom_frame:
            goal_left_buffer = PointStamped(goal_parts.header, left_post)
            goal_right_buffer = PointStamped(goal_parts.header, right_post)
            try:
                self.goal_odom.left_post = self.tf_buffer.transform(goal_left_buffer, self.odom_frame,
                                                                    timeout=rospy.Duration(0.2)).point
                self.goal_odom.right_post = self.tf_buffer.transform(goal_right_buffer, self.odom_frame,
                                                                     timeout=rospy.Duration(0.2)).point
                self.goal_odom.header.frame_id = self.odom_frame
                self.goal_seen_time = rospy.Time.now()
            except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
                rospy.logwarn(e)
        else:
            self.goal_odom.left_post = left_post
            self.goal_odom.right_post = right_post
            self.goal_seen_time = rospy.Time.now()
        self.goal_publisher.publish(self.goal_odom.to_pose_with_certainty_array())

    ###########
    # ## Pose #
    ###########

    def pose_callback(self, pos: PoseWithCovarianceStamped):
        self.pose = pos

    def get_current_position(self):
        """
        Returns the current position as determined by the localization
        """
        try:
            # get the most recent transform
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_footprint_frame, rospy.Time(0))
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
            rospy.logwarn(e)
            return None
        orientation = transform.transform.rotation
        theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        return transform.transform.translation.x, transform.transform.translation.y, theta

    def get_localization_precision(self):
        """
        Returns the current localization precision based on the covariance matrix.
        """
        x_sdev = self.pose.pose.covariance[0]  # position 0,0 in a 6x6-matrix
        y_sdev = self.pose.pose.covariance[7]  # position 1,1 in a 6x6-matrix
        theta_sdev = self.pose.pose.covariance[35]  # position 5,5 in a 6x6-matrix
        return (x_sdev, y_sdev, theta_sdev)

    def localization_precision_in_threshold(self) -> bool:
        """
        Returns whether the last localization precision values were in the threshold defined in the settings.
        """
        # Check whether we received a message in the last pose_lost_time seconds.
        if not self.localization_pose_current():
            return False
        # get the standard deviation values of the covariance matrix
        precision = self.get_localization_precision()
        # return whether those values are in the threshold
        return precision[0] < self.pose_precision_threshold['x_sdev'] and \
               precision[1] < self.pose_precision_threshold['y_sdev'] and \
               precision[2] < self.pose_precision_threshold['theta_sdev']

    def localization_pose_current(self) -> bool:
        """
        Returns whether the last localization pose was received in the last pose_lost_time-setting seconds.
        """
        return rospy.Time.now() - self.pose.header.stamp < rospy.Duration.from_sec(self.pose_lost_time)


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

    ############
    # Obstacle #
    ############

    def global_costmap_callback(self, msg):
        self.local_obstacle_map = msg
        self.local_obstacle_map_numpy = ros_numpy.numpify(self.local_obstacle_map)
        self.add_obstacles_costmap()

    def global_costmap_update_callback(self, msg):
        data_np = np.array(msg.data, dtype=np.int8).reshape(msg.height, msg.width)
        self.local_obstacle_map_numpy[msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = data_np
        self.add_obstacles_costmap()

    def add_obstacles_costmap(self):
        scale = 1 / self.local_obstacle_map.info.resolution / 2
        obstacle = self.local_obstacle_map_numpy.T[
                    int(self.local_obstacle_map_numpy.shape[0] / 2 - 9 * scale) : int(self.local_obstacle_map_numpy.shape[0] / 2 + 9 * scale) : 2,
                    int(self.local_obstacle_map_numpy.shape[1] / 2 - 6 * scale) : int(self.local_obstacle_map_numpy.shape[1] / 2 + 6 * scale) : 2].astype(np.float) / 100 * 0.4
        x, y = self.get_ball_position_xy()
        x = int((x - 4.5) * 10)
        y = int((y - 3) * 10)
        obstacle[x - 2 : x + 2, y - 2: y + 2] = 0
        self.costmap = self.base_costmap.copy() + gaussian_filter(obstacle, 3)
        self.calc_gradients()

    def calc_gradients(self):
        gradient = np.gradient(self.base_costmap)
        norms = np.linalg.norm(gradient,axis=0)
        gradient = [np.where(norms==0,0,i/norms) for i in gradient]
        self.gradient_map = gradient

    def cost_at_relative_xy(self, x, y):
        if self.costmap is None:
            return 0.0

        point = PointStamped()
        point.header.stamp = rospy.Time(0)
        point.header.frame_id = self.base_footprint_frame
        point.point.x = x
        point.point.y = y

        try:
            # Transform point of interest to the map
            point = self.tf_buffer.transform(point, self.map_frame, timeout=rospy.Duration(0.3))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            rospy.logwarn(e)
            return 0.0

        return self.get_cost_at_field_position(point.point.x, point.point.y)


    def calc_gradient_map(self):

        goal_width = rospy.get_param("goal_width", 2)
        field_width = rospy.get_param("field_width", 6)
        field_length = rospy.get_param("field_length", 9)
        goal_factor = 0.2
        keep_out_border = 0.2
        in_field_value_our_side = 0.8
        in_field_value_enemy_side = 0.4

        # Create Grid
        grid_x, grid_y = np.mgrid[0:field_length:field_length*10j, 0:field_width:field_width*10j]

        fix_points = []

        # Add base points
        fix_points.extend([
            # Corner points of the field
            [[0,            0],             1],
            [[field_length, 0],             1],
            [[0,            field_width],   1],
            [[field_length, field_width],   1],
            # Points in the field that pull the gradient down, so we don't play always in the middle
            [[keep_out_border,                  keep_out_border],                 in_field_value_our_side],
            #[[field_length - keep_out_border,   keep_out_border],                 in_field_value_enemy_side],
            [[keep_out_border,                  field_width - keep_out_border],   in_field_value_our_side],
            #[[field_length - keep_out_border,   field_width - keep_out_border],   in_field_value_enemy_side]
        ])

        # Add goal area (including the dangerous parts on the side of the goal)
        fix_points.extend([
            [[field_length, field_width/2 - goal_width/2],                      0.4],
            [[field_length, field_width/2 - goal_width/2 + goal_factor],    0],
            [[field_length, field_width/2 + goal_width/2 - goal_factor],    0],
            [[field_length, field_width/2 + goal_width/2],                      0.4],
        ])

        # Interpolate the keypoints from above to form the costmap
        interpolated = griddata([p[0] for p in fix_points], [p[1] for p in fix_points], (grid_x, grid_y), method='linear')

        # Smooth the costmap to get more continus gradients
        self.base_costmap = gaussian_filter(interpolated, 4)
        self.costmap = self.base_costmap.copy()

        self.calc_gradients()

        return
        # Viz
        plt.quiver(grid_x, grid_y, -gradient[0], -gradient[1])
        plt.show()
        plt.imshow(self.base_costmap.T, origin='lower')
        plt.show()
        plt.imshow(interpolated.T, origin='lower')
        plt.show()

    def get_gradient_at_field_position(self, x, y):
        """
        Gets the gradient tuple at a given field position
        :param x: Field coordiante in the x direction
        :param y: Field coordiante in the y direction
        """
        field_width = rospy.get_param("field_width", 6)
        field_length = rospy.get_param("field_length", 9)
        x_map = int(min((field_length * 10)-1, max(0, (x + field_length / 2) * 10)))
        y_map = int(min((field_width * 10)-1,max(0, (y + field_width / 2) * 10)))
        return -self.gradient_map[0][x_map, y_map], -self.gradient_map[1][x_map, y_map]

    def get_cost_at_field_position(self, x, y):
        """
        Gets the costmap value at a given field position
        :param x: Field coordinate in the x direction
        :param y: Field coordinate in the y direction
        """
        field_width = rospy.get_param("field_width", 6)
        field_length = rospy.get_param("field_length", 9)
        x_map = int(min((field_length * 10)-1, max(0, (x + field_length / 2) * 10)))
        y_map = int(min((field_width * 10)-1,max(0, (y + field_width / 2) * 10)))

        #plt.imshow(self.costmap[x_map - 20 : x_map + 20, y_map - 20 : y_map + 20], origin='lower')
        #plt.show()
        return self.costmap[x_map, y_map]

    def get_gradient_direction_at_field_position(self, x, y):
        """
        Returns the gradient direction at the given position
        :param x: Field coordiante in the x direction
        :param y: Field coordiante in the y direction
        """
        if self.costmap.sum() > 0 and False:
            goal_width = rospy.get_param("goal_width", 2)
            field_width = rospy.get_param("field_width", 6)
            field_length = rospy.get_param("field_length", 9)
            goal_factor = 0.2
            keep_out_border = 0.2
            in_field_value_our_side = 0.8
            in_field_value_enemy_side = 0.2

            # Create Grid
            grid_x, grid_y = np.mgrid[0:field_length:field_length*10j, 0:field_width:field_width*10j]
            plt.imshow(self.costmap.T, origin='lower')
            plt.show()
            plt.quiver(grid_x, grid_y, -self.gradient_map[0], -self.gradient_map[1])
            plt.show()
        grad = self.get_gradient_at_field_position(x, y)
        return math.atan2(grad[1], grad[0])