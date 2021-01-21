#!/usr/bin/env python3

import rospy
import tf2_ros

from geometry_msgs.msg import Pose, Vector3, PointStamped, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from humanoid_league_msgs.msg import PoseWithCertaintyArray
from std_msgs.msg import String


class ShowWorldModelObjects:
    """
    This class provides RViZ markers to visualize the position of the goal and the goalposts as known to the world model
    of the behavior. Furthermore the ball_kick_are is published as RViZ Marker.
    """
    def __init__(self):
        rospy.init_node("show_world_model_objects")
        self.marker_publisher = rospy.Publisher("visualization_world_model_markers", Marker, queue_size=10)

        # object properties
        self.ball_diameter = 0.17
        self.lifetime_ball = rospy.get_param('behavior/body/ball_lost_time')
        self.lifetime_goal = rospy.get_param('behavior/body/goal_lost_time')
        self.lifetime_ball_kick_area = 1
        self.post_diameter = 0.15
        self.post_height = 1.10
        position = Pose()
        position.position.x = 1.0
        position.position.y = 1.0
        position.position.z = 0.0
        position.orientation.w = 1.0
        position.orientation.x = 0.0
        position.orientation.y = 0.0
        position.orientation.z = 0.0

        # init ball marker
        self.marker_ball = Marker()  # type: Marker
        self.marker_ball.id = 0
        self.marker_ball.type = Marker.SPHERE
        self.ball_pose = Pose()
        scale = Vector3(self.ball_diameter, self.ball_diameter, self.ball_diameter)
        self.marker_ball.scale = scale
        self.ball_color = ColorRGBA()
        self.ball_color.a = 0.0
        self.ball_color.g = 1.0
        self.marker_ball.color = self.ball_color
        self.marker_ball.pose = position
        self.marker_ball.lifetime = rospy.Duration(self.lifetime_ball)

        # init goal markers
        self.marker_goal_first_post = Marker()  # type:Marker
        self.marker_goal_first_post.id = 1
        self.marker_goal_first_post.type = Marker.CYLINDER
        self.goal_post_pose = Pose()
        scale = Vector3(self.post_diameter, self.post_diameter, self.post_height)
        self.marker_goal_first_post.scale = scale
        self.post_left_color = ColorRGBA()
        self.post_left_color.a = 0.0
        self.post_left_color.g = 1.0
        self.marker_goal_first_post.color = self.post_left_color
        self.marker_goal_first_post.pose = position
        self.marker_goal_first_post.lifetime = rospy.Duration(self.lifetime_goal)

        self.marker_goal_second_post = Marker()  # type:Marker
        self.marker_goal_second_post.id = 2
        self.marker_goal_second_post.type = Marker.CYLINDER
        self.goal_post_pose = Pose()
        scale = Vector3(self.post_diameter, self.post_diameter, self.post_height)
        self.marker_goal_second_post.scale = scale
        self.post_right_color = ColorRGBA()
        self.post_right_color.a = 0.0
        self.post_right_color.b = 1.0
        self.marker_goal_second_post.color = self.post_right_color
        self.marker_goal_second_post.pose = position
        self.marker_goal_second_post.lifetime = rospy.Duration(self.lifetime_goal)

        # init ball kick area markers
        self.marker_kick_area_right = Marker()
        self.marker_kick_area_right.id = 3
        self.marker_kick_area_right.type = Marker.LINE_STRIP
        self.marker_kick_area_right.scale.x = 0.01
        self.kick_area_right_color = ColorRGBA()
        self.kick_area_right_color.a = 0.5
        self.kick_area_right_color.r = 0.8
        self.kick_area_right_color.g = 0.8
        self.kick_area_right_color.b = 0.8
        self.marker_kick_area_right.color = self.kick_area_right_color
        self.marker_kick_area_right.lifetime = rospy.Duration(self.lifetime_ball_kick_area)
        self.marker_kick_area_right.pose.orientation.w = 1.0

        self.marker_kick_area_left = Marker()
        self.marker_kick_area_left.id = 4
        self.marker_kick_area_left.type = Marker.LINE_STRIP
        self.marker_kick_area_left.scale.x = 0.01
        self.kick_area_left_color = ColorRGBA()
        self.kick_area_left_color.a = 0.5
        self.kick_area_left_color.r = 0.8
        self.kick_area_left_color.g = 0.8
        self.kick_area_left_color.b = 0.8
        self.marker_kick_area_left.color = self.kick_area_left_color
        self.marker_kick_area_left.lifetime = rospy.Duration(self.lifetime_ball_kick_area)
        self.marker_kick_area_left.pose.orientation.w = 1.0

        # init subscribers
        rospy.Subscriber("debug/viz_ball", PointStamped, self.ball_cb, queue_size=10)
        rospy.Subscriber("debug/viz_goal", PoseWithCertaintyArray, self.goal_cb, queue_size=10)
        rospy.Subscriber("debug/viz_ball_kick_area", String, self.kick_area_cb, queue_size=10)

        # load kick area info
        kick_right_max_x = rospy.get_param('behavior/body/right_kick_max_x')
        kick_right_max_y = rospy.get_param('behavior/body/right_kick_max_y')
        kick_right_min_x = rospy.get_param('behavior/body/right_kick_min_x')
        kick_right_min_y = rospy.get_param('behavior/body/right_kick_min_y')
        kick_left_max_x = rospy.get_param('behavior/body/left_kick_max_x')
        kick_left_max_y = rospy.get_param('behavior/body/left_kick_max_y')
        kick_left_min_x = rospy.get_param('behavior/body/left_kick_min_x')
        kick_left_min_y = rospy.get_param('behavior/body/left_kick_min_y')
        self.kick_area_info = [[kick_right_max_x, kick_right_max_y, kick_right_min_x, kick_right_min_y],
                               [kick_left_max_x, kick_left_max_y, kick_left_min_x, kick_left_min_y]]

        # init tf listener for ball_kick_area_viz
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        self.last_received_kick_info = rospy.Time.now()

        self.kick_area()
        rospy.spin()

    def kick_area(self):
        """
        This method updates the pose of the markers of the ball_kick_areas according to the current location of the
        base_footprint and sets back the color of the markers from green to grey when the last received message from the
        kick_ball_area decision is older than one second.
        :return: None
        """
        while not rospy.is_shutdown():
            # This is right now not necessary as the ball_kick_area is defined relative to the base_footprint.
            try:
                current_base_link_pose = self.tfBuffer.lookup_transform('base_footprint', 'base_footprint',
                                                                        rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                continue

            # set stamp and frame_id
            self.marker_kick_area_right.header.stamp = rospy.Time.now()
            self.marker_kick_area_right.header.frame_id = 'base_footprint'
            self.marker_kick_area_left.header.stamp = rospy.Time.now()
            self.marker_kick_area_left.header.frame_id = 'base_footprint'

            # set corners of the two square markers
            kick_areas = []
            for i in range(2):
                point1 = Point()
                point1.x = current_base_link_pose.transform.translation.x + self.kick_area_info[i][0]
                point1.y = current_base_link_pose.transform.translation.y + self.kick_area_info[i][1]
                point2 = Point()
                point2.x = current_base_link_pose.transform.translation.x + self.kick_area_info[i][0]
                point2.y = current_base_link_pose.transform.translation.y + self.kick_area_info[i][3]
                point3 = Point()
                point3.x = current_base_link_pose.transform.translation.x + self.kick_area_info[i][2]
                point3.y = current_base_link_pose.transform.translation.y + self.kick_area_info[i][3]
                point4 = Point()
                point4.x = current_base_link_pose.transform.translation.x + self.kick_area_info[i][2]
                point4.y = current_base_link_pose.transform.translation.y + self.kick_area_info[i][1]
                kick_area = [point1, point2, point3, point4, point1]
                kick_areas.append(kick_area)
            self.marker_kick_area_right.points = kick_areas[0]
            self.marker_kick_area_left.points = kick_areas[1]

            # set color back to grey
            if abs(self.last_received_kick_info - rospy.Time.now()) > rospy.Duration(1):
                self.marker_kick_area_right.color.r = 0.8
                self.marker_kick_area_right.color.g = 0.8
                self.marker_kick_area_right.color.b = 0.8
                self.marker_kick_area_left.color.r = 0.8
                self.marker_kick_area_left.color.g = 0.8
                self.marker_kick_area_left.color.b = 0.8
            self.marker_publisher.publish(self.marker_kick_area_right)
            self.marker_publisher.publish(self.marker_kick_area_left)

    def ball_cb(self, msg):
        self.marker_ball.header = msg.header
        self.marker_ball.pose.position = msg.point
        self.marker_ball.color.a = 0.5
        self.marker_publisher.publish(self.marker_ball)

    def goal_cb(self, msg):
        if len(msg.poses) > 0:
            self.marker_goal_first_post.header = msg.header
            self.marker_goal_first_post.pose.position = msg.poses[0].pose.pose.position
            self.marker_goal_first_post.pose.position.z = 0.5 * self.post_height
            self.marker_goal_first_post.color.a = 0.5
            self.marker_publisher.publish(self.marker_goal_first_post)
        if len(msg.poses) > 1:
            self.marker_goal_second_post.header = msg.header
            self.marker_goal_second_post.pose.position = msg.poses[1].pose.pose.position
            self.marker_goal_second_post.pose.position.z = 0.5 * self.post_height
            self.marker_goal_second_post.color.a = 0.5
            self.marker_publisher.publish(self.marker_goal_second_post)

    def kick_area_cb(self, msg):
        # set the marker for the right ball_kick_area to green and the left one to red as the behavior decided the ball
        # is within this area
        if msg.data == 'RIGHT':
            self.marker_kick_area_right.color.r = 0.0
            self.marker_kick_area_right.color.g = 1.0
            self.marker_kick_area_right.color.b = 0.0
            self.marker_kick_area_left.color.r = 1.0
            self.marker_kick_area_left.color.g = 0.0
            self.marker_kick_area_left.color.b = 0.0
        # set the marker for the left ball_kick_area to green and the right one to red as the behavior decided the ball
        # is within this area
        elif msg.data == 'LEFT':
            self.marker_kick_area_left.color.r = 0.0
            self.marker_kick_area_left.color.g = 1.0
            self.marker_kick_area_right.color.b = 0.0
            self.marker_kick_area_right.color.r = 1.0
            self.marker_kick_area_right.color.g = 0.0
            self.marker_kick_area_left.color.b = 0.0
        # set the color of both markers to red as the behavior decided the ball is not within one of the areas
        else:
            self.marker_kick_area_right.color.r = 1.0
            self.marker_kick_area_right.color.g = 0.0
            self.marker_kick_area_right.color.b = 0.0
            self.marker_kick_area_left.color.r = 1.0
            self.marker_kick_area_left.color.g = 0.0
            self.marker_kick_area_left.color.b = 0.0
        self.last_received_kick_info = rospy.Time.now()


if __name__ == "__main__":
    marker_node = ShowWorldModelObjects()
