#!/usr/bin/env python3
"""
ObstaclePublisher
^^^^^^^^^^^^^^^^^

This node publishes the ball and other obstacles as an obstacle to avoid walking through it
"""
import rospy
from humanoid_league_msgs.msg import ObstacleRelativeArray, PoseWithCertainty, PoseWithCertaintyArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_srvs.srv import Empty
from std_msgs.msg import Header, Bool


class ObstaclePublisher:
    def __init__(self):
        rospy.init_node("obstacle_publisher")
        self.clearer = rospy.ServiceProxy("move_base/clear_costmaps", Empty)
        rospy.logwarn("Waiting for clear_costmap service")
        self.clearer.wait_for_service()
        rospy.logwarn("Found Service clear_costmap")

        rospy.Subscriber("balls_relative", PoseWithCertaintyArray, self._balls_callback, queue_size=1)
        rospy.Subscriber("ball_obstacle_active", Bool, self._ball_active_callback, queue_size=1)
        rospy.Subscriber("obstacles_relative", ObstacleRelativeArray, self._obstacle_callback, queue_size=1)

        self.obstacle_publisher = rospy.Publisher("obstacles", PointCloud2, queue_size=10)

        self.publish_timer = rospy.Timer(rospy.Duration(1/20), self.publish_obstacles)

        self.balls = []
        self.balls_header = None
        self.ball_active = True

        self.obstacles = []
        self.obstacles_header = None

        rospy.spin()

    def publish_obstacles(self, event):

        # Publish balls
        if self.ball_active:
            for ball in self.balls:  # TODO timespamp aware and transfroms
                self.obstacle_publisher.publish(create_cloud_xyz32(self.balls_header, [[ball.pose.pose.position.x, ball.pose.pose.position.y, ball.pose.pose.position.z]]))

        # Publish other obstacles
        distance = 0.05  # TODO remove after time not new message
        points = []
        for o in self.obstacles:
            points.append([o.pose.pose.pose.position.x, o.pose.pose.pose.position.y, o.pose.pose.pose.position.z])
            points.append([o.pose.pose.pose.position.x - distance, o.pose.pose.pose.position.y - distance, o.pose.pose.pose.position.z])
            points.append([o.pose.pose.pose.position.x - distance, o.pose.pose.pose.position.y + distance, o.pose.pose.pose.position.z])
            points.append([o.pose.pose.pose.position.x + distance, o.pose.pose.pose.position.y - distance, o.pose.pose.pose.position.z])
            points.append([o.pose.pose.pose.position.x + distance, o.pose.pose.pose.position.y + distance, o.pose.pose.pose.position.z])
        self.obstacle_publisher.publish(create_cloud_xyz32(self.obstacles_header, points))

    def _balls_callback(self, msg):
        self.balls = msg.poses
        self.balls_header = msg.header

    def _obstacle_callback(self, msg):
        self.obstacles = msg.obstacles
        self.obstacles_header = msg.header

    def _ball_active_callback(self, msg):
        self.ball_active = msg.data

if __name__ == "__main__":
    ObstaclePublisher()
