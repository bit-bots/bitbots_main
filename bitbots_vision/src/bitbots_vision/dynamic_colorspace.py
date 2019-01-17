#! /usr/bin/env python2


import rospy
import rospkg

class DynamicColorspace:

    def __init__(self):
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('bitbots_dynamic_colorspace')

        rospy.init_node('bitbots_dynmaic_colorspace')
        rospy.loginfo('Initializing dynmaic colorspace...')

        self.config = {}

        rospy.spin()
