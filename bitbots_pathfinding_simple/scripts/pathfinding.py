#!/usr/bin/env python
import rospy
import tf2_ros as tf2
from geometry_msgs.msg import Twist, PointStamped, Quaternion
from humanoid_league_msgs.msg import Position2D
from tf.transformations import quaternion_from_euler

class Pathfinding:
    def __init__(self):
        rospy.init_node('pathfinding_simple')
        rospy.Subscriber('/bitbots_pathfinding/relative_goal', Position2D, self.goal_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.max_front = 0.05
        self.max_left = 0.03
        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(5))
        self.tf_listener = tf2.TransformListener(self.tf_buffer)
        print(2)
        rospy.spin()

    def goal_callback(self, msg):
        # type msg: Position2D
        # Transform to base footprint
        stamped = PointStamped()
        stamped.header = msg.header
        stamped.point.x = msg.pose.x
        stamped.point.y = msg.pose.y

        try:
            position = self.tf_buffer.transform(stamped, 'base_footprint', timeout=rospy.Duration(0.3))
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException) as e:
            rospy.logwarn(e)
            return

        cmd_vel = Twist()
        if position.point.x == 0 and position.point.y == 0:
            cmd_vel.angular.z = msg.pose.theta # TODO this is in the wrong frame
            self.cmd_pub.publish(cmd_vel)
            rospy.sleep(2.0)
            cmd_vel = Twist()
            self.cmd_pub.publish(cmd_vel)
            return
        scale = position.point.x / 0.05 if position.point.x > 0.05 else 1
        rospy.loginfo(scale)
        cmd_vel.linear.x = max(min(position.point.x / scale, self.max_front), -self.max_front)
        cmd_vel.linear.y = max(min(position.point.y / scale, self.max_left), -self.max_left)
        self.cmd_pub.publish(cmd_vel)


if __name__ == "__main__":
    Pathfinding()
