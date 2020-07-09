#!/usr/bin/env python3

import rospy
from bitbots_msgs.msg import FootPressure
from bitbots_msgs.srv import FootScale, FootScaleRequest, FootScaleResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

CALIBRATION_WEIGHT = 0.850 * 9.81

rospy.init_node("pressure_calibration")


rospy.loginfo("Waiting for " + "/foot_pressure_left/set_foot_zero" + "/foot_pressure_left/set_foot_scale" 
              + "/foot_pressure_right/set_foot_zero" +"/foot_pressure_right/set_foot_scale")

rospy.wait_for_service("/foot_pressure_left/set_foot_zero")
rospy.wait_for_service("/foot_pressure_left/set_foot_scale")
rospy.wait_for_service("/foot_pressure_right/set_foot_zero")
rospy.wait_for_service("/foot_pressure_right/set_foot_scale")
rospy.loginfo("found all services")

rospy.loginfo("Welcome to the foot calibration suite. ")
rospy.loginfo("default calibration weight is {} kg. Change the code when using a different weight.".format(CALIBRATION_WEIGHT))


rospy.loginfo("Press enter when there is no load on the weight cells of the LEFT foot")
input()


zero = rospy.ServiceProxy("/foot_pressure_left/set_foot_zero", Empty)
if zero():
    rospy.loginfo("Successfully set the zero position for all sensors")
else:
    rospy.loginfo("There was an error :(")
    exit(1)



sensors = ['left front','left back','right front','right back',]

scale = rospy.ServiceProxy("/foot_pressure_left/set_foot_scale", FootScale)
request = FootScaleRequest()
request.weight = CALIBRATION_WEIGHT

rospy.loginfo("Starting calibration for LEFT foot")
for i in range(len(sensors)):
    request.sensor = i
    rospy.loginfo("Place calibration weight on " + sensors[i] + " cleat. Then press enter.")
    input()
    scale(request)
    rospy.loginfo("Load cell for " + sensors[i] + " calibrated")



rospy.loginfo("Press enter when there is no load on the weight cells of the RIGHT foot")
input()
zero = rospy.ServiceProxy("/foot_pressure_right/set_foot_zero", Empty)
if zero():
    rospy.loginfo("Successfully set the zero position for all sensors")
else:
    rospy.loginfo("There was an error :(")
    exit(1)

scale = rospy.ServiceProxy("/foot_pressure_right/set_foot_scale", FootScale)

rospy.loginfo("Starting calibration for RIGHT foot")
for i in range(len(sensors)):
    request.sensor = i
    rospy.loginfo("Place calibration weight on " + sensors[i] + " cleat. Then press enter.")
    input()
    scale(request)
    rospy.loginfo("Load cell for " + sensors[i] + " calibrated")


rospy.loginfo("Thank you for your patience. The configuration has been automatically saved by the pressure_converter already")
rospy.loginfo("Shutting down now.")
