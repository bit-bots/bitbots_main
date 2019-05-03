#!/usr/bin/env python

import rospy
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class GoalConverter:
    def __init__(self):
        rospy.init_node('goal_converter')
        self.goal_subscriber = rospy.Subscriber('behavior/goal', PoseStamped, self.goal_callback)
        self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.spin()

    def goal_callback(self, msg):
        # type msg: PoseStamped
        if msg.header.frame_id == 'map':
            self.goal_publisher.publish(msg)
        else:
            try:
                msg.header.stamp = rospy.Time(0)
                map_goal = self.tf_buffer.transform(msg, 'map', timeout=rospy.Duration(0.5))
                e = euler_from_quaternion((map_goal.pose.orientation.x, map_goal.pose.orientation.y,
                                           map_goal.pose.orientation.z, map_goal.pose.orientation.w))
                q = quaternion_from_euler(0, 0, e[2])
                map_goal.pose.orientation.x = q[0]
                map_goal.pose.orientation.y = q[1]
                map_goal.pose.orientation.z = q[2]
                map_goal.pose.orientation.w = q[3]
                map_goal.pose.position.z = 0
                self.goal_publisher.publish(map_goal)
            except Exception as e:
                rospy.logwarn(e)


if __name__ == '__main__':
    GoalConverter()