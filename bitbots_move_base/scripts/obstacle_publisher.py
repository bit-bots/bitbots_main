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
from std_msgs.msg import Header


class ObstaclePublisher:
    def __init__(self):
        rospy.init_node("obstacle_publisher")
        self.clearer = rospy.ServiceProxy("move_base/clear_costmaps", Empty)
        rospy.logwarn("Waiting for clear_costmap service")
        self.clearer.wait_for_service()
        rospy.logwarn("Found Service clear_costmap")

        rospy.Subscriber("balls_relative", PoseWithCertaintyArray, self._balls_callback, queue_size=1)
        rospy.Subscriber("obstacles_relative", ObstacleRelativeArray, self._obstacle_callback, queue_size=1)

        self.obstacle_publisher = rospy.Publisher("obstacles", PointCloud2, queue_size=10)

        rospy.spin()

    def _balls_callback(self, msg):
        balls = msg.poses
        for ball in balls:
            self.obstacle_publisher.publish(create_cloud_xyz32(msg.header, [[ball.pose.pose.position.x, ball.pose.pose.position.y, ball.pose.pose.position.z]]))

    def _obstacle_callback(self, msg):
        self.obstacle_publisher.publish(create_cloud_xyz32(msg.header, \
            [[o.pose.pose.pose.position.x, o.pose.pose.pose.position.y, o.pose.pose.pose.position.z] for o in msg.obstacles]))

if __name__ == "__main__":
    ObstaclePublisher()
