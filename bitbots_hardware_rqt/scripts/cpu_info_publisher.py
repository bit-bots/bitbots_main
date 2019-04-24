#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
import rospy, rospkg, os
from bitbots_msgs.msg import Cpu
import subprocess
import yaml

send_delay = 2

def cpu_info_publisher():
    """Publishes the cpu temperature, name and usage (in percent) of the maschine its running on"""
    mpstaterr = False
    rospy.init_node('cpu_info', anonymous=True)
    cpu_pub = rospy.Publisher('cpu_info', Cpu, queue_size=10)
    rate = rospy.Rate(send_delay) # hz
    while not rospy.is_shutdown():
        cpu_temp = str(subprocess.check_output(['bash', '-c', "cat /sys/class/thermal/thermal_zone*/temp"]))
        cpu_temp_str = cpu_temp[0:2]+'.'+cpu_temp[2] #
        cpu_usage = str(subprocess.check_output(['bash', '-c', "mpstat"]))
        msg = Cpu()
        if cpu_usage.startswith("Linux"): # checks if the bash output is from mpstat, logs info if not
            cpu_usage_array =cpu_usage.split()
            msg.cpu_usage = cpu_usage_array[21].replace(',','.') # mpstat output entry 21 is the entry of the cpu usage, replaces , with .
        else:
            if not mpstaterr:
                rospy.logerr("mpstat is not installed! cpu usage can not be published")
                mpstaterr = True
            msg.cpu_usage = ""
        msg.cpu_name = rospy.get_param('~cpu_name')
        msg.temperature = cpu_temp_str
        cpu_pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
     try:
        cpu_info_publisher()
     except rospy.ROSInterruptException:
         pass
