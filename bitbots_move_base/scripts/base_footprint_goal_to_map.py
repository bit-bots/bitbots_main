#!/usr/bin/env python

import rospy
import tf2_ros
from tf2_geometry_msgs import PoseStamped

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
                map_goal.pose.orientation.x = 0
                map_goal.pose.orientation.y = 0
                map_goal.pose.orientation.z = 0
                map_goal.pose.orientation.w = 1
                self.goal_publisher.publish(map_goal)
            except Exception as e:
                rospy.logwarn(e)


if __name__ == '__main__':
    GoalConverter()