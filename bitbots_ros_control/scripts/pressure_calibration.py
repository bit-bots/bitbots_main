#!/usr/bin/env python3

import rospy
from bitbots_msgs.msg import FootPressure
from bitbots_msgs.srv import FootScale, FootScaleRequest, FootScaleResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

rospy.init_node("pressure_calibration")
rospy.loginfo("Welcome to the foot calibration suite. Press enter when there is no load on the weight cells")
input()
rospy.loginfo("waiting for service /set_foot_zero")
rospy.wait_for_service("/set_foot_zero")
zero = rospy.ServiceProxy("/set_foot_zero", Empty)
if zero():
    rospy.loginfo("Successfully set the zero position for all sensors")
else:
    rospy.loginfo("There was an error :(")
    exit(1)

sensors = ['left foot, left front', 'left foot, left back', 'left foot, right front','left foot, right back',
           'right foot, left front','right foot, left back','right foot, right front','right foot, right back',]
scale = rospy.ServiceProxy("/set_foot_scale", FootScale)
request = FootScaleRequest()
request.weight = 0.810
rospy.loginfo("default calibration weight is 0.81 kg. Change the code when using a different weight.")
for i in range(len(sensors)):
    request.sensor = i
    rospy.loginfo("Place calibration weight on " + sensors[i] + " cleat. Then press enter.")
    input()
    scale(request)
    rospy.loginfo("Load cell for " + sensors[i] + " calibrated")

rospy.loginfo("Thank you for your patience. The configuration has been automatically saved by the pressure_converter already")
rospy.loginfo("Shutting down now.")
