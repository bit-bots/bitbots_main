#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Pose, Vector3, PointStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from humanoid_league_msgs.msg import GoalRelative


class ShowWorldModelObjects:
    """
    This class provides RViZ markers to visualize the position of the goal and the goalposts as known to the world model
     of the behavior.
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

        # init subscribers
        rospy.Subscriber("/debug/viz_ball", PointStamped, self.ball_cb, queue_size=10)
        rospy.Subscriber("/debug/viz_goal", GoalRelative, self.goal_cb, queue_size=10)

        rospy.spin()

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


if __name__ == "__main__":
    marker_node = ShowWorldModelObjects()
