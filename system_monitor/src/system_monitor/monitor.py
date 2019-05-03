#!/usr/bin/env python
import rospy

from system_monitor import msg as SystemMonitorMsg

if __name__ == '__main__':
    rospy.init_node('system_monitor')
    rospy.spin()
