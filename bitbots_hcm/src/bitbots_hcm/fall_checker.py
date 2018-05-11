# -*- coding: utf8 -*-
import math
import numpy
import rospy
from tf.transformations import euler_from_quaternion

class FallChecker(object):
    def __init__(self):
        # will be set by dynamic reconfigure
        robot_type_name = rospy.get_param("robot_type_name")

        # Fallanimation laden
        self.falling_motor_degrees_front = rospy.get_param(
            "hcm/falling/falling_front")
        self.falling_motor_degrees_back = rospy.get_param(
            "hcm/falling/falling_back")
        self.falling_motor_degrees_right = rospy.get_param(
            "hcm/falling/falling_right")
        self.falling_motor_degrees_left = rospy.get_param(
            "hcm/falling/falling_left")

        # load config values depending on robot type and lode them into param server to set
        # start values for dynamic reconfigure
        # There are no default values set in dynamic reconfigure, in order to make it dependable on the robot
        self.dyn_falling_active = rospy.get_param("hcm/falling/" + robot_type_name + "/dyn_falling_active")
        rospy.set_param("hcm/dyn_falling_active", self.dyn_falling_active)
        self.ground_coefficient = rospy.get_param("hcm/falling/" + robot_type_name + "/ground_coefficient")
        rospy.set_param("hcm/ground_coefficient", self.ground_coefficient)

        if not rospy.has_param("ZMPConfig/" + robot_type_name + "/HipPitch"):
            rospy.logwarn("HipPitch offset from walking was not found on parameter server, will use 0.")
        self.falling_threshold_front = rospy.get_param("hcm/falling/" + robot_type_name + "/threshold_gyro_y_front") \
                                       + math.radians(rospy.get_param("ZMPConfig/" + robot_type_name + "/HipPitch", -10))
        rospy.set_param("hcm/threshold_gyro_y_front", self.falling_threshold_front)
        self.falling_threshold_back = rospy.get_param("hcm/falling/" + robot_type_name + "/threshold_gyro_y_back") \
                                      + math.radians(rospy.get_param("ZMPConfig/" + robot_type_name + "/HipPitch", 10))
        rospy.set_param("hcm/threshold_gyro_y_back", self.falling_threshold_back)
        self.falling_threshold_right = rospy.get_param("hcm/falling/" + robot_type_name + "/threshold_gyro_x_right")
        rospy.set_param("hcm/threshold_gyro_x_right", self.falling_threshold_right)
        self.falling_threshold_left = rospy.get_param("hcm/falling/" + robot_type_name + "/threshold_gyro_x_left")
        rospy.set_param("hcm/threshold_gyro_x_left", self.falling_threshold_left)

        # Grenzwerte an Untergrund anpassen
        self.falling_threshold_front *= self.ground_coefficient
        self.falling_threshold_back *= self.ground_coefficient
        self.falling_threshold_right *= self.ground_coefficient
        self.falling_threshold_left *= self.ground_coefficient

        # Neue Werte für die Detectierung des Fallens, anhand der momentanen Lage
        self.falling_threshold_orientation_front_back = 20 
        self.falling_threshold_orientation_left_right = 20
        

    def update_reconfigurable_values(self, config, level):
        self.dyn_falling_active = config["dyn_falling_active"]
        self.ground_coefficient = config["ground_coefficient"]
        self.falling_threshold_front = config["threshold_gyro_y_front"]
        self.falling_threshold_back = config["threshold_gyro_y_back"]
        self.falling_threshold_right = config["threshold_gyro_x_right"]
        self.falling_threshold_left = config["threshold_gyro_x_left"]

    def check_falling(self, not_much_smoothed_gyro, quaternion):
        """Checks if the robot is currently falling and in which direction. """
        # First decide if we fall more sidewards or more front-back-wards. Then decide if we fall badly enough
        # to do something about it
        euler = euler_from_quaternion(quaternion)
        self.check_falling_x_y(not_much_smoothed_gyro)

    def check_fall(not_much_smoothed_gyro, euler):
        x_value = 0
        y_value = 0

        if  numpy.sign(euler[0]) == numpy.sign(not_much_smoothed_gyro[0]:
            skalar = (self.falling_threshold_orientation_left_right - abs(euler[0]))/self.falling_threshold_orientation_left_right
            if self.falling_threshold_back * skalar < abs(not_much_smoothed_gyro[0]:
                x_value = abs(not_much_smoothed_gyro[0]) * (1-skalar)

        if numpy.sign(euler[1]) == numpy.sign(self.not_much_smoothed_gyro[1]:
            skalar = (self.falling_threshold_orietation_front_back - abs(euler[1]))/self.falling_threshold_orientation_front_back
            if self.falling_threshold_left * skalar < abs(not_much_smoothed_gyro[1]:
                y_value = abs(not_much_smoothed_gyro[1]) * (1-skalar)

        if x_value + y_value == 0:
            return None
        
        if y_value > x_value:
            if not_much_smoothed_gyro[1] > 0:
                rospy.logdebug("FALLING TO THE FRONT")
                return self.falling_motor_degrees_front
            else:
                rospy.logdebug("FALLING TO THE BACK")
                return self.falling_motor_degrees_back
        else:
            if not_much_smoothed_gyro[0] > 0:
                rospy.logdebug("FALLING TO THE RIGHT")
                return self.falling_motor_degrees_right
            else:
                rospy.logdebug("FALLING TO THE LEFT")
                return self.falling_motor_degrees_left


    def check_fallen(self, smooth_accel):
        """Check if the robot has fallen and is lying on the floor. Returns animation to play, if necessary."""
        if smooth_accel[0] > 7:
            rospy.logdebug("Lying on belly, should stand up")
            return rospy.get_param("hcm/animations/front-up")

        if smooth_accel[0] < -7:
            rospy.logdebug("Lying on my back, should stand up!")
            return rospy.get_param("hcm/animations/bottom-up")

        if abs(smooth_accel[1]) > 7:
            rospy.logdebug("Lying on the side, should stand up. Trying to stand up from front. Is that right?")
            return rospy.get_param("hcm/animations/front-up")

        return None
