#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D


class Pathfinding:
    def __init__(self):
        rospy.init_node('pathfinding_simple')
        rospy.Subscriber('/bitbots_pathfinding/relative_goal', Pose2D, self.goal_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.max_front = 0.05
        self.max_left = 0.03
        rospy.spin()

    def goal_callback(self, msg: Pose2D):
        cmd_vel = Twist()
        if msg.x == 0 and msg.y == 0:
            cmd_vel.angular.z = msg.theta
            self.cmd_pub.publish(cmd_vel)
            rospy.sleep(2.0)
            cmd_vel = Twist()
            self.cmd_pub.publish(cmd_vel)
            return
        scale = msg.x / 0.05 if msg.x > 0.05 else 1
        rospy.loginfo(scale)
        cmd_vel.linear.x = max(min(msg.x / scale, self.max_front), -self.max_front)
        cmd_vel.linear.y = max(min(msg.y / scale, self.max_left), -self.max_left)
        self.cmd_pub.publish(cmd_vel)


if __name__ == "__main__":
    Pathfinding()
