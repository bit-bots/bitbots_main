#!/usr/bin/env python3
"""
ObstaclePublisher
^^^^^^^^^^^^^^^^^

This node publishes the ball and other obstacles as an obstacle to avoid walking through it
"""
import rospy
import ros_numpy
import numpy as np
import tf2_ros as tf2
import tf2_geometry_msgs
from humanoid_league_msgs.msg import ObstacleRelativeArray, PoseWithCertainty, PoseWithCertaintyArray, TeamData
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
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

        self.tf_buffer = tf2.Buffer(rospy.Duration(5))
        self.tf_listener = tf2.TransformListener(self.tf_buffer)

        self.map_frame = rospy.get_param('~map_frame', 'map')

        rospy.Subscriber("ball_position_relative_filtered", PoseWithCovarianceStamped, self._balls_callback,
                         queue_size=1)
        rospy.Subscriber("ball_obstacle_active", Bool, self._ball_active_callback, queue_size=1)
        rospy.Subscriber("obstacles_relative", ObstacleRelativeArray, self._obstacle_callback, queue_size=1)
        rospy.Subscriber("team_data", TeamData, self._team_data_callback, queue_size=1)

        self.robot_obstacle_publisher = rospy.Publisher("robot_obstacles", PointCloud2, queue_size=10)
        self.ball_obstacle_publisher = rospy.Publisher("ball_obstacles", PointCloud2, queue_size=10)

        self.ball = None
        self.ball_active = True

        self.robots = []

        self.team = dict()

        self.robots_storage_time = 10
        self.robot_merge_distance = 0.5

        rospy.Timer(rospy.Duration(1/20), self.publish_obstacles)

        rospy.spin()

    def publish_obstacles(self, event):
        # Set current timespamp and frame
        dummy_header = Header()
        dummy_header.stamp = rospy.Time.now()
        dummy_header.frame_id = self.map_frame

        # Publish balls
        if self.ball_active and self.ball is not None:
            self.ball_obstacle_publisher.publish(
                create_cloud_xyz32(dummy_header, [[self.ball.point.x, self.ball.point.y, self.ball.point.z]]))

        # Cleanup robot obstacles
        self.robots = list(filter(
            lambda robot: abs((rospy.Time.now() - robot.header.stamp).to_sec()) < self.robots_storage_time,
            self.robots))

        # Enlarge robots
        width = 0.2
        points = []
        for o in self.robots:
            points.append([o.point.x, o.point.y, o.point.z])
            points.append([o.point.x - width / 2, o.point.y - width / 2, o.point.z])
            points.append([o.point.x - width / 2, o.point.y + width / 2, o.point.z])
            points.append([o.point.x + width / 2, o.point.y - width / 2, o.point.z])
            points.append([o.point.x + width / 2, o.point.y + width / 2, o.point.z])

        # Publish team mates
        width = 0.3
        team_points = points   # This is only equal for now. This changed if our robots are published seperatly.
        for robot in self.team.values():
            p = robot.robot_position.pose.position
            team_points.append([p.x, p.y, p.z])
            team_points.append([p.x - width / 2, p.y - width / 2, p.z])
            team_points.append([p.x - width / 2, p.y + width / 2, p.z])
            team_points.append([p.x + width / 2, p.y - width / 2, p.z])
            team_points.append([p.x + width / 2, p.y + width / 2, p.z])
        self.robot_obstacle_publisher.publish(create_cloud_xyz32(dummy_header, team_points))

    def _balls_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame,
                                                        msg.header.frame_id,
                                                        msg.header.stamp,
                                                        rospy.Duration(1.0))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            rospy.logwarn(e)
            return
        point = PointStamped()
        point.header = msg.header
        point.point = msg.pose.pose.position
        point = tf2_geometry_msgs.do_transform_point(point, transform)
        self.ball = point

    def _obstacle_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame,
                                                        msg.header.frame_id,
                                                        msg.header.stamp,
                                                        rospy.Duration(1.0))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            rospy.logwarn(e)
            return

        for o in msg.obstacles:
            point = PointStamped()
            point.header = msg.header
            point.point = o.pose.pose.pose.position
            # Transfrom robot to map frame
            point = tf2_geometry_msgs.do_transform_point(point, transform)
            # Update robots that are close together
            cleaned_robots = []
            for old_robot in self.robots:
                # Check if there is another robot in memory close to it
                if np.linalg.norm(ros_numpy.numpify(point.point) - ros_numpy.numpify(old_robot.point)) > self.robot_merge_distance:
                    cleaned_robots.append(old_robot)
            # Update our robot list with a new list that does not contain the duplicates
            self.robots = cleaned_robots
            # Append our new robots
            self.robots.append(point)

    def _ball_active_callback(self, msg):
        self.ball_active = msg.data

    def _team_data_callback(self, msg):
        self.team[msg.robot_id] = msg


if __name__ == "__main__":
    ObstaclePublisher()
