#!/usr/bin/env python

import rospy
import tf2_ros

from geometry_msgs.msg import Pose, Vector3, PointStamped, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from humanoid_league_msgs.msg import GoalRelative
from std_msgs.msg import String


class ShowWorldModelObjects:
    """
    This class provides RViZ markers to visualize the position of the goal and the goalposts as known to the world model
     of the behavior. Furthermore the ball_kick_are is published as RViZ Marker.
    """
    def __init__(self):
        rospy.init_node("show_world_model_objects")
        self.marker_publisher = rospy.Publisher("/visualization_world_model_markers", Marker, queue_size=10)

        # object properties
        self.ball_diameter = 0.17
        self.lifetime = int(5 * (10 ** 9))
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
        self.marker_ball.lifetime = rospy.Duration(nsecs=self.lifetime)

        # init goal markers
        self.marker_goal_left = Marker()  # type:Marker
        self.marker_goal_left.id = 1
        self.marker_goal_left.type = Marker.CYLINDER
        self.goal_post_pose = Pose()
        scale = Vector3(self.post_diameter, self.post_diameter, self.post_height)
        self.marker_goal_left.scale = scale
        self.post_left_color = ColorRGBA()
        self.post_left_color.a = 0.0
        self.post_left_color.g = 1.0
        self.marker_goal_left.color = self.post_left_color
        self.marker_goal_left.pose = position
        self.marker_goal_left.lifetime = rospy.Duration(nsecs=self.lifetime)

        self.marker_goal_right = Marker()  # type:Marker
        self.marker_goal_right.id = 2
        self.marker_goal_right.type = Marker.CYLINDER
        self.goal_post_pose = Pose()
        scale = Vector3(self.post_diameter, self.post_diameter, self.post_height)
        self.marker_goal_right.scale = scale
        self.post_right_color = ColorRGBA()
        self.post_right_color.a = 0.0
        self.post_right_color.b = 1.0
        self.marker_goal_right.color = self.post_right_color
        self.marker_goal_right.pose = position
        self.marker_goal_right.lifetime = rospy.Duration(nsecs=self.lifetime)

        # init ball kick area markers
        self.marker_kick_area_right = Marker()
        self.marker_kick_area_right.id = 3
        self.marker_kick_area_right.type = Marker.LINE_STRIP
        self.marker_kick_area_right.scale.x = 0.01
        self.kick_area_right_color = ColorRGBA()
        self.kick_area_right_color.a = 1.0
        self.kick_area_right_color.r = 1.0
        self.marker_kick_area_right.color = self.kick_area_right_color
        self.marker_kick_area_right.lifetime = rospy.Duration(nsecs=self.lifetime)
        self.marker_kick_area_right.pose.orientation.w = 1.0

        self.marker_kick_area_left = Marker()
        self.marker_kick_area_left.id = 4
        self.marker_kick_area_left.type = Marker.LINE_STRIP
        self.marker_kick_area_left.scale.x = 0.01
        self.kick_area_left_color = ColorRGBA()
        self.kick_area_left_color.a = 1.0
        self.kick_area_left_color.r = 1.0
        self.marker_kick_area_left.color = self.kick_area_left_color
        self.marker_kick_area_left.lifetime = rospy.Duration(nsecs=self.lifetime)
        self.marker_kick_area_left.pose.orientation.w = 1.0

        self.last_received_kick_info = rospy.Time.now()

        # init subscribers
        rospy.Subscriber("/debug/viz_ball", PointStamped, self.ball_cb, queue_size=10)
        rospy.Subscriber("/debug/viz_goal", GoalRelative, self.goal_cb, queue_size=10)
        rospy.Subscriber("/debug/viz_ball_kick_area", String, self.kick_area_cb, queue_size=10)

        # load kick area info
        self.kick_right_max_x = rospy.get_param('behavior/body/right_kick_max_x')
        self.kick_right_max_y = rospy.get_param('behavior/body/right_kick_max_y')
        self.kick_right_min_x = rospy.get_param('behavior/body/right_kick_min_x')
        self.kick_right_min_y = rospy.get_param('behavior/body/right_kick_min_y')
        self.kick_left_max_x = rospy.get_param('behavior/body/left_kick_max_x')
        self.kick_left_max_y = rospy.get_param('behavior/body/left_kick_max_y')
        self.kick_left_min_x = rospy.get_param('behavior/body/left_kick_min_x')
        self.kick_left_min_y = rospy.get_param('behavior/body/left_kick_min_y')

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        self.kick_area()
        rospy.spin()

    def kick_area(self):
        while not rospy.is_shutdown():
            try:
                current_base_link_pose = self.tfBuffer.lookup_transform('base_footprint', 'base_footprint', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                continue
            self.marker_kick_area_right.header.stamp = rospy.Time.now()
            self.marker_kick_area_right.header.frame_id = 'base_footprint'
            self.marker_kick_area_left.header.stamp = rospy.Time.now()
            self.marker_kick_area_left.header.frame_id = 'base_footprint'
            point1 = Point()
            point1.x = current_base_link_pose.transform.translation.x + self.kick_right_max_x
            point1.y = current_base_link_pose.transform.translation.y + self.kick_right_max_y
            point2 = Point()
            point2.x = current_base_link_pose.transform.translation.x + self.kick_right_max_x
            point2.y = current_base_link_pose.transform.translation.y + self.kick_right_min_y
            point3 = Point()
            point3.x = current_base_link_pose.transform.translation.x + self.kick_right_min_x
            point3.y = current_base_link_pose.transform.translation.y + self.kick_right_min_y
            point4 = Point()
            point4.x = current_base_link_pose.transform.translation.x + self.kick_right_min_x
            point4.y = current_base_link_pose.transform.translation.y + self.kick_right_max_y
            self.marker_kick_area_right.points = [point1, point2, point3, point4,
                                                  point1]
            point5 = Point()
            point5.x = current_base_link_pose.transform.translation.x + self.kick_left_max_x
            point5.y = current_base_link_pose.transform.translation.y + self.kick_left_max_y
            point6 = Point()
            point6.x = current_base_link_pose.transform.translation.x + self.kick_left_max_x
            point6.y = current_base_link_pose.transform.translation.y + self.kick_left_min_y
            point7 = Point()
            point7.x = current_base_link_pose.transform.translation.x + self.kick_left_min_x
            point7.y = current_base_link_pose.transform.translation.y + self.kick_left_min_y
            point8 = Point()
            point8.x = current_base_link_pose.transform.translation.x + self.kick_left_min_x
            point8.y = current_base_link_pose.transform.translation.y + self.kick_left_max_y
            self.marker_kick_area_left.points = [point5, point6, point7, point8,
                                                  point5]
            if abs(self.last_received_kick_info - rospy.Time.now()) > rospy.Duration(1):
                self.marker_kick_area_right.color.r = 1.0
                self.marker_kick_area_right.color.g = 0.0
                self.marker_kick_area_left.color.r = 1.0
                self.marker_kick_area_left.color.g = 0.0

            self.marker_publisher.publish(self.marker_kick_area_right)
            self.marker_publisher.publish(self.marker_kick_area_left)

    def ball_cb(self, msg):
        self.marker_ball.header = msg.header
        self.marker_ball.pose.position = msg.point
        self.marker_ball.color.a = 0.5
        self.marker_publisher.publish(self.marker_ball)

    def goal_cb(self, msg):
        self.marker_goal_left.header = msg.header
        self.marker_goal_left.pose.position = msg.left_post
        self.marker_goal_left.pose.position.z = 0.5 * self.post_height
        self.marker_goal_left.color.a = 0.5
        self.marker_goal_right.header = msg.header
        self.marker_goal_right.pose.position = msg.right_post
        self.marker_goal_right.pose.position.z = 0.5 * self.post_height
        self.marker_goal_right.color.a = 0.5
        self.marker_publisher.publish(self.marker_goal_left)
        self.marker_publisher.publish(self.marker_goal_right)

    def kick_area_cb(self, msg):
        if msg.data == 'RIGHT':
            self.marker_kick_area_right.color.r = 0.0
            self.marker_kick_area_right.color.g = 1.0
            self.marker_kick_area_left.color.r = 1.0
            self.marker_kick_area_left.color.g = 0.0
        elif msg.data == 'LEFT':
            self.marker_kick_area_left.color.r = 0.0
            self.marker_kick_area_left.color.g = 1.0
            self.marker_kick_area_right.color.r = 1.0
            self.marker_kick_area_right.color.g = 0.0
        else:
            self.marker_kick_area_right.color.r = 1.0
            self.marker_kick_area_right.color.g = 0.0
            self.marker_kick_area_left.color.r = 1.0
            self.marker_kick_area_left.color.g = 0.0
        self.last_received_kick_info = rospy.Time.now()


if __name__ == "__main__":
    marker_node = ShowWorldModelObjects()
