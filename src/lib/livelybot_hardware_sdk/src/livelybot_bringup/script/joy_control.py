#!/usr/bin/env python
# coding: utf-8

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Teleop:
    def __init__(self):
        rospy.init_node('joy_teleop', anonymous=True)
        
        # 从参数服务器获取参数，如果没有设置则使用默认值
        self.axis_lin_x = rospy.get_param('~axis_linear_x', 1)
        self.axis_lin_y = rospy.get_param('~axis_linear_y', 0)
        self.axis_ang = rospy.get_param('~axis_angular', 6)
        self.vlinear = rospy.get_param('~vel_linear', 0.15)
        self.vangular = rospy.get_param('~vel_angular', 0.2)
        self.config_vlinear = rospy.get_param('~config vel',0)
        self.config_vangular = rospy.get_param('~config vel',1)
        self.ton = rospy.get_param('~button', 5)
        
        # 设置发布者和订阅者
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('joy', Joy, self.callback)
    
    def callback(self, joy):
        twist = Twist()
        
        if joy.buttons[self.config_vlinear]:
            self.vlinear = joy.axes[4]
            rospy.loginfo("vlinear: %.3f", self.vlinear)
        if joy.buttons[self.config_vangular]:
            self.vangular = joy.axes[4]
            rospy.loginfo("vangular: %.3f", self.vangular)

        if joy.buttons[self.ton]:

            twist.linear.x = joy.axes[self.axis_lin_x] * self.vlinear
            twist.linear.y = joy.axes[self.axis_lin_y] * self.vlinear
            twist.angular.z = joy.axes[self.axis_ang] * self.vangular
            rospy.loginfo("linear x y: %.3f %.3f angular: %.3f", twist.linear.x, twist.linear.y, twist.angular.z)
            self.pub.publish(twist)

if __name__ == '__main__':
    try:
        teleop = Teleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
