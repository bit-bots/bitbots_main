#!/usr/bin/env python2.7
# -*- coding:utf-8 -*-
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyNode(object):
    """ This node handles pressing of buttons on the robot. It should be used to call services on other nodes,
    as an sort of event driven architecture for the buttons.
    """

    def __init__(self):
        log_level = rospy.DEBUG if rospy.get_param("/debug_active", False) else rospy.INFO
        rospy.init_node("joy_to_twist", log_level=log_level, anonymous=False)

        # --- Initialize Topics ---
        rospy.Subscriber("/joy", Joy, self.joy_cb)
        self.walk_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.walk_msg = Twist()

        self.walk_msg.linear.x = 0.0;
        self.walk_msg.linear.y = 0.0;
        self.walk_msg.linear.z = 0.0;
        
        self.walk_msg.angular.x = 0.0;
        self.walk_msg.angular.y = 0.0;
        self.walk_msg.angular.z = 0.0;


        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.walk_publisher.publish(self.walk_msg)
            r.sleep()

    def joy_cb(self, msg):
        if msg.axes[1] > 0:
            self.walk_msg.linear.x = 0.05 * msg.axes[1]
        else:
            self.walk_msg.linear.x = 0.03 * msg.axes[1]

        self.walk_msg.linear.y = 0.03 * msg.axes[0]


        self.walk_msg.angular.z = -0.05 * msg.axes[2]

if __name__ == "__main__":
    joy = JoyNode()
    rospy.spin()
