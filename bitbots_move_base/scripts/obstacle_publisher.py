#!/usr/bin/env python2
"""
ObstaclePublisher
^^^^^^^^^^^^^^^^^

This node publishes the ball and other obstacles as an obstacle to avoid walking through it
"""
import rospy
from humanoid_league_msgs.msg import BallRelative, ObstaclesRelative
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

        rospy.Subscriber("ball_relative", BallRelative, self._ball_callback, queue_size=1)
        rospy.Subscriber("obstacles_relative", ObstaclesRelative, self._obstacle_callback, queue_size=1)

        self.obstacle_publisher = rospy.Publisher("obstacles", PointCloud2, queue_size=10)

        self.ball = None
        self.obstacles = None
        r = rospy.Rate(3.0)

        while not rospy.is_shutdown():
            self.clearer()
            rospy.sleep(0.1)

            obs = list()
            if self.ball is not None:
                obs.append([self.ball.ball_relative.x, self.ball.ball_relative.y, self.ball.ball_relative.z])

            if self.obstacles is not None:
                obs.extend([o.position.x, o.position.y, o.position.z] for o in self.obstacles.obstacles)

            if self.obstacles is not None or self.ball is not None:
                h = Header()
                h.stamp = rospy.get_rostime()
                # TODO check if frameids are the same
                if self.ball is not None:
                    h.frame_id = self.ball.header.frame_id
                else:
                    h.frame_id = self.obstacles.header.frame_id
                self.obstacle_publisher.publish(create_cloud_xyz32(h, obs))
            r.sleep()

    def _ball_callback(self, msg):
        self.ball = msg

    def _obstacle_callback(self, msg):
        self.obstacles = msg

if __name__ == "__main__":
    ObstaclePublisher()
