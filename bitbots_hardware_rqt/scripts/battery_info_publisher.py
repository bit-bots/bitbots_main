#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy, rospkg, os
from hardware_info.msg import BatteryMessage
import subprocess


DELAY_TOKEN = "SEND_DELAY"

send_delay = 2 

def battery_info_publisher():
    """Publishes battery voltage, current and max energy to the node battery_info."""

    rospy.init_node('battery_info', anonymous=True)
    battery_pub = rospy.Publisher('battery_info', BatteryMessage, queue_size=10)
    rate = rospy.Rate(send_delay) # hz
    while not rospy.is_shutdown():
        battery_voltage = subprocess.check_output(['bash', '-c', "cat /sys/class/power_supply/BAT1/voltage_now"])
        battery_energy_full = subprocess.check_output(['bash', '-c', "cat /sys/class/power_supply/BAT1/energy_full"])
        battery_energy_now = subprocess.check_output(['bash', '-c', "cat /sys/class/power_supply/BAT1/energy_now"])
        msg = BatteryMessage()
        msg.voltage = battery_voltage[0:2] + '.' + battery_voltage[3] #reformat microvolt into volt
        msg.charge_percentage = str(float(battery_energy_now) / float(battery_energy_full) * 100)
        battery_pub.publish(msg)
        rate.sleep()

def readIpConfig():
    rp = rospkg.RosPack()
    ip_filename = os.path.join(rp.get_path('hardware_info'), 'resource', 'ip_config.yaml')


    with open(ip_filename, "r") as file:
        ip_config = yaml.load(file)
        send_delay = float(ip_config.get(DELAY_TOKEN)) / 1000.0
        file.close()

if __name__ == '__main__':
     try:
        readIpConfig()
        battery_info_publisher()
     except rospy.ROSInterruptException:
         pass
