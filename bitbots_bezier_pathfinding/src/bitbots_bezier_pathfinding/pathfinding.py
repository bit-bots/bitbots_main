#!/usr/bin/env python
import math
import numpy
import rospy
import tf_conversions

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose2D, Pose, PoseStamped, Twist
from nav_msgs.msg import Path

from bitbots_bezier_pathfinding.cfg import PathfindingConfig
from bitbots_bezier_pathfinding.bezier import Bezier

class BezierPathfinding:
    def __init__(self):
        rospy.init_node('bezier_pathfinding')
        Server(PathfindingConfig, self.dynamic_reconfigure_callback)
        rospy.spin()

    def dynamic_reconfigure_callback(self, config, level):
        # ROS Publishers/Subscribers
        self.path_publisher = rospy.Publisher(config['path_publisher'], Path, queue_size=1)
        self.command_pose_publisher = rospy.Publisher(config['command_pose_publisher'], PoseStamped, queue_size=1)
        self.command_publisher = rospy.Publisher(config['command_publisher'], Twist, queue_size=1)
        rospy.Subscriber(config['goal_subscriber'], PoseStamped, self.goal_callback)

        self.straightness = config['straightness']
        self.stepsize = config['stepsize']
        self.velocity = config['velocity']
        self.planning_time = config['planning_time']

        return config

    def goal_callback(self, goal_msg):
        goal_pose = Pose2D()
        goal_pose.x = goal_msg.pose.position.x
        goal_pose.y = goal_msg.pose.position.y
        angle_quaternion = (goal_msg.pose.orientation.x,
                            goal_msg.pose.orientation.y,
                            goal_msg.pose.orientation.z,
                            goal_msg.pose.orientation.w)
        angle_euler = tf_conversions.transformations.euler_from_quaternion(angle_quaternion)
        goal_pose.theta = angle_euler[2]  # Yaw of the euler angle
        curve = Bezier.from_pose(Pose2D(0, 0, 0), goal_pose, self.straightness)
        self.publish_curve_as_path(curve)
        self.publish_cmd_vel(curve)

    def publish_curve_as_path(self, bezier_curve):
        ts = numpy.linspace(0, 1, 50)
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'base_link'
        path_poses = []
        for t in ts:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'base_link'
            pose.pose.position.x, pose.pose.position.y = bezier_curve.get_xy(t)
            pose.pose.orientation.w = 1
            path_poses.append(pose)
        path_msg.poses = path_poses
        self.path_publisher.publish(path_msg)

    def publish_command_pose(self, x, y, theta):
        # Publish the pose where the next cmd_vel is going
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Euler -> Quaternion (a lot simpler without roll and pitch)
        pose.pose.orientation.z = math.sin(0.5 * theta)
        pose.pose.orientation.w = math.cos(0.5 * theta)
        self.command_pose_publisher.publish(pose)


    def publish_cmd_vel(self, bezier_curve):
        # Where am I after self.planning_time seconds?
        t = bezier_curve.get_time_at_distance(self.velocity * self.planning_time, self.stepsize)
        x, y = bezier_curve.get_xy(t)
        theta = bezier_curve.get_direction(t, self.stepsize)
        self.publish_command_pose(x, y, theta)
        # theta is the angle that I will have turned until then
        # therefore, my angular velocity is theta / planning_time
        angular_z = theta / self.planning_time
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.velocity
        cmd_vel_msg.angular.z = angular_z
        self.command_publisher.publish(cmd_vel_msg)


if __name__ == '__main__':
    BezierPathfinding()
