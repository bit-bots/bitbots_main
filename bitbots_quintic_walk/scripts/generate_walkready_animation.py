#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from bitbots_msgs.msg import JointCommand
import math


def callback(msg):
    output_string = "{\n \"author\": \"walkready_script\", \n \"description\": \"none\",\n \"hostname\": \"tams159\", \n \"keyframes\": [ \n { \n \"duration\": 1.0, \n \"goals\": { \n"
    output_string += " \"HeadPan\": 0, \n \"HeadTilt\": 0, \n \"LElbow\": 45, \n \"LShoulderPitch\": 0, \n \"LShoulderRoll\": 0, \n \"RElbow\": -45, \n \"RShoulderPitch\": 0, \n \"RShoulderRoll\": 0,"
    i = 0
    for joint_name in msg.joint_names:
        output_string += " \"" + str(joint_name) + "\" : " + str(math.degrees(msg.positions[i]))
        if i < len(msg.joint_names) -1:
            output_string += ","
        output_string += "\n" 
        i +=1    
    output_string += "},\n \"name\": \"generated frame\", \n \"pause\": 0.5 \n }\n ], \n \"name\": \"walkready\", \n\"version\": 0 \n} \n"
    with open("walkready.json", "w") as text_file:
        text_file.write(output_string)



rospy.init_node("walkready_script")
rospy.logwarn("Make sure that the QuinticWalk is running but no other nodes or this script will not work.")

cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
rospy.Subscriber("/walking_motor_goals", JointCommand, callback)
rospy.sleep(1)

# get the robot to walk
twist = Twist()
twist.linear.x = 0.3
cmd_pub.publish(twist)
rospy.sleep(1)
# stop the walk
twist.linear.x = 0
cmd_pub.publish(twist)
rospy.sleep(3)
# now we have recorded the right joint positions, save 

rospy.loginfo("Your walkready animation has been saved to the current directory.")






