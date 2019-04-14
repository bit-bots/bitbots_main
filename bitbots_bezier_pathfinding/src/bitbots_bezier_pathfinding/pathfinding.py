#!/usr/bin/env python
import math
import numpy
import rospy
import tf2_ros

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose2D, Twist
from tf2_geometry_msgs import PoseStamped, do_transform_pose
from nav_msgs.msg import Path

from bitbots_bezier_pathfinding.cfg import PathfindingConfig
from bitbots_bezier_pathfinding.bezier import Bezier


class BezierPathfinding:
    def __init__(self):
        rospy.init_node('bezier_pathfinding')
        Server(PathfindingConfig, self.dynamic_reconfigure_callback)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.timer = None  # type: rospy.Timer

        rospy.spin()

    def dynamic_reconfigure_callback(self, config, level):
        # ROS Publishers/Subscribers
        self.path_publisher = rospy.Publisher(config['path_publisher'], Path, queue_size=1)
        self.command_pose_publisher = rospy.Publisher(config['command_pose_publisher'], PoseStamped, queue_size=1)
        self.command_publisher = rospy.Publisher(config['command_publisher'], Twist, queue_size=1)
        self.goal_republisher = rospy.Publisher(config['goal_subscriber'], PoseStamped, queue_size=2)
        rospy.Subscriber(config['goal_subscriber'], PoseStamped, self.goal_callback)

        self.straightness = config['straightness']
        self.stepsize = config['stepsize']
        self.velocity = config['velocity']
        self.planning_time = config['planning_time']
        self.refresh_time = config['refresh_time']
        self.stop_distance = config['stop_distance']

        return config

    def absolute_to_relative(self, pose):
        """Convert a PoseStamped to a PoseStamped in base_footprint"""
        self.frame_id = pose.header.frame_id
        if pose.header.frame_id == 'base_footprint':
            return pose
        elif pose.header.frame_id == 'map' or pose.header.frame_id == 'odom':
            try:
                return self.tf_buffer.transform(pose, 'base_footprint', timeout=rospy.Duration(0.3))
            except tf2_ros.LookupException as e:
                rospy.logwarn('A goal was published in {} frame but this frame does not exist'.format(self.frame_id))
                return
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn('A goal was published in {} frame but this frame is no longer published'.format(self.frame_id))
                return
        else:
            raise AssertionError('goal_msg must be in base_footprint of map frame')

    def relative_to_absolute(self, pose):
        """Convert a PoseStamped to a PoseStamped in self.frame_id"""
        if pose.header.frame_id == self.frame_id:
            return pose
        else:
            try:
                return self.tf_buffer.transform(pose, self.frame_id, timeout=rospy.Duration(0.3))
            except tf2_ros.LookupException:
                rospy.logwarn('Command velocities have been created in frame {} but this frame was never published'.format(self.frame_id))
                return
            except tf2_ros.ExtrapolationException:
                rospy.logwarn('Command velocities have been created in frame {} but this frame is no longer available'.format(self.frame_id))
                return

    def pose_to_pose2d(self, pose):
        """Convert a PoseStamped message to a Pose2D message"""
        assert pose.header.frame_id == 'base_footprint'
        pose2d = Pose2D()
        pose2d.x = pose.pose.position.x
        pose2d.y = pose.pose.position.y
        pose2d.theta = math.atan2(2 * pose.pose.orientation.z * pose.pose.orientation.w,
                                  1 - 2 * pose.pose.orientation.z * pose.pose.orientation.z)
        return pose2d

    def pose2d_to_pose(self, pose2d):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'base_footprint'
        pose.pose.position.x = pose2d.x
        pose.pose.position.y = pose2d.y
        # Euler -> Quaternion (a lot simpler without roll and pitch)
        pose.pose.orientation.z = math.sin(0.5 * pose2d.theta)
        pose.pose.orientation.w = math.cos(0.5 * pose2d.theta)
        return pose

    def goal_callback(self, goal_msg):
        if self.timer:
            self.timer.shutdown()
        # convert goal_msg to a relative Pose2D
        goal_msg.header.stamp = rospy.Time.now()
        goal_pose = self.absolute_to_relative(goal_msg)
        if goal_pose:
            goal_pose_2d = self.pose_to_pose2d(goal_pose)

            # own position (relative) is always 0 0 0
            own_pose_2d = Pose2D(0, 0, 0)

            curve = Bezier.from_pose(own_pose_2d, goal_pose_2d, self.straightness)
            self.publish_curve_as_path(curve)
            self.publish_cmd_vel(curve)

        if self.refresh_time > 0:
            self.timer = rospy.Timer(rospy.Duration(self.refresh_time), lambda event: self.goal_callback(goal_msg), oneshot=True)

    def publish_curve_as_path(self, bezier_curve):
        ts = numpy.linspace(0, 1, 50)
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.frame_id
        path_poses = []
        # Transform to absolute position
        try:
            transform = self.tf_buffer.lookup_transform(self.frame_id, 'base_footprint', rospy.Time.now(), rospy.Duration(0.3))
        except Exception as e:
            rospy.logwarn(e)
            return

        for t in ts:
            x, y = bezier_curve.get_xy(t)
            pose = self.pose2d_to_pose(Pose2D(x, y, 0))
            pose = do_transform_pose(pose, transform)
            path_poses.append(pose)

        path_msg.poses = path_poses
        self.path_publisher.publish(path_msg)

    def publish_command_pose(self, pose_2d):
        # Publish the pose where the next cmd_vel is going
        pose = self.pose2d_to_pose(pose_2d)
        pose = self.relative_to_absolute(pose)
        self.command_pose_publisher.publish(pose)

    def publish_cmd_vel(self, bezier_curve):
        # Where am I after self.planning_time seconds?
        t = bezier_curve.get_time_at_distance(self.velocity * self.planning_time, self.stepsize)
        distance = bezier_curve.get_distance_at_time(1, self.stepsize)
        if distance < self.stop_distance:
            # Stop the walking
            cmd_vel_msg = Twist()
        else:
            x, y = bezier_curve.get_xy(t)
            theta = bezier_curve.get_direction(t, self.stepsize)

            # Debug output
            pose_2d = Pose2D(x, y, theta)
            self.publish_command_pose(pose_2d)

            # theta is the angle that I will have turned until then
            # therefore, my angular velocity is theta / planning_time
            angular_z = theta / self.planning_time
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.velocity
            cmd_vel_msg.angular.z = angular_z
        self.command_publisher.publish(cmd_vel_msg)


if __name__ == '__main__':
    BezierPathfinding()
