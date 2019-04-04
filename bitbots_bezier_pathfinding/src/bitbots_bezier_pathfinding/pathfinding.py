#!/usr/bin/env python
import math
import numpy
import rospy
import tf2_ros

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose2D, Pose, PoseStamped, Twist
from nav_msgs.msg import Path
from tf2_geometry_msgs import PoseStamped as TFPoseStamped

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

    def goal_callback(self, goal_msg):
        if self.timer:
            self.timer.shutdown()
        # goal_msg must be either in base_footprint or map frame
        goal_pose = Pose2D()
        goal_pose.x = goal_msg.pose.position.x
        goal_pose.y = goal_msg.pose.position.y
        # Quaternion -> Euler (a lot simpler because x and y are guaranteed to be 0)
        yaw = math.atan2(2 * goal_msg.pose.orientation.z * goal_msg.pose.orientation.w,
                         1 - 2 * goal_msg.pose.orientation.z * goal_msg.pose.orientation.z)
        goal_pose.theta = yaw
        if goal_msg.header.frame_id == 'base_footprint':
            self.frame_id = 'base_footprint'
            own_pose = Pose2D(0, 0, 0)
        elif goal_msg.header.frame_id == 'map' or goal_msg.header.frame_id == 'odom':
            self.frame_id = goal_msg.header.frame_id
            # Get own position from tf to map frame
            own_pose_relative = TFPoseStamped()
            own_pose_relative.header.frame_id = 'base_footprint'
            own_pose_relative.header.stamp = rospy.Time.now()
            own_pose_relative.pose.orientation.w = 1
            try:
                own_pose_3d = self.tf_buffer.transform(own_pose_relative, self.frame_id, timeout=rospy.Duration(0.3))
            except tf2_ros.LookupException:
                rospy.logwarn('A goal was published in {} frame but this frame does not exist'.format(self.frame_id))
                return
            except tf2_ros.ExtrapolationException:
                rospy.logwarn('A goal was published in {} frame but this frame is no longer published'.format(self.frame_id))
                return
            own_pose = Pose2D()
            own_pose.x = own_pose_3d.pose.position.x
            own_pose.y = own_pose_3d.pose.position.y
            # Quaternion -> Euler again
            yaw = math.atan2(2 * own_pose_3d.pose.orientation.z * own_pose_3d.pose.orientation.w,
                             1 - 2 * own_pose_3d.pose.orientation.z * own_pose_3d.pose.orientation.z)
            own_pose.theta = yaw
        else:
            raise AssertionError('goal_msg must be in base_footprint of map frame')

        curve = Bezier.from_pose(own_pose, goal_pose, self.straightness)
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
        for t in ts:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x, pose.pose.position.y = bezier_curve.get_xy(t)
            pose.pose.orientation.w = 1
            path_poses.append(pose)
        path_msg.poses = path_poses
        self.path_publisher.publish(path_msg)

    def publish_command_pose(self, x, y, theta):
        # Publish the pose where the next cmd_vel is going
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        # Euler -> Quaternion (a lot simpler without roll and pitch)
        pose.pose.orientation.z = math.sin(0.5 * theta)
        pose.pose.orientation.w = math.cos(0.5 * theta)
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
            self.publish_command_pose(x, y, theta)
            if self.frame_id == 'base_footprint':
                theta_relative = theta
            else:
                # cmd vel is relative to the robot, therefore a transform is necessary
                pose_absolute = TFPoseStamped()
                pose_absolute.header.frame_id = self.frame_id
                pose_absolute.header.stamp = rospy.Time.now()
                pose_absolute.pose.position.x = x
                pose_absolute.pose.position.y = y
                # Euler -> Quaternion again
                pose_absolute.pose.orientation.z = math.sin(0.5 * theta)
                pose_absolute.pose.orientation.w = math.cos(0.5 * theta)
                try:
                    pose_relative_stamped = self.tf_buffer.transform(pose_absolute, 'base_footprint', timeout=rospy.Duration(0.3))
                except tf2_ros.LookupException:
                    rospy.logwarn('Command velocities have been created in frame {} but this frame was never published'.format(self.frame_id))
                    return
                except tf2_ros.ExtrapolationException:
                    rospy.logwarn('Command velocities have been created in frame {} but this frame is no longer available'.format(self.frame_id))
                    return
                # Quaternion -> Euler again
                theta_relative = math.atan2(2 * pose_relative_stamped.pose.orientation.z * pose_relative_stamped.pose.orientation.w,
                                   1 - 2 * pose_relative_stamped.pose.orientation.z * pose_relative_stamped.pose.orientation.z)
            # theta is the angle that I will have turned until then
            # therefore, my angular velocity is theta / planning_time
            angular_z = theta_relative / self.planning_time
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.velocity
            cmd_vel_msg.angular.z = angular_z
        self.command_publisher.publish(cmd_vel_msg)


if __name__ == '__main__':
    BezierPathfinding()
