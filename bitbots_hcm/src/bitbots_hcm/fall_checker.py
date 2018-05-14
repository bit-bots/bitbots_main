# -*- coding: utf8 -*-
import math
import numpy
import rospy

class FallChecker(object):
    def __init__(self):

        # will be set by dynamic reconfigure
        robot_type_name = rospy.get_param("robot_type_name")
        # print("Enter Check init 1")
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
        # There are no default vaules set in dynamic reconfigure, in order to make it dependable on the robot
        self.dyn_falling_active = rospy.get_param("hcm/falling/" + robot_type_name + "/dyn_falling_active")
        rospy.set_param("hcm/dyn_falling_active", self.dyn_falling_active)
        self.ground_coefficient = rospy.get_param("hcm/falling/" + robot_type_name + "/ground_coefficient")
        rospy.set_param("hcm/ground_coefficient", self.ground_coefficient)

        if not rospy.has_param("ZMPConfig/" + robot_type_name + "/HipPitch"):
            rospy.logwarn("HipPitch offset from walking was not found on parameter server, will use 0.")
        self.falling_threshold_front = rospy.get_param("hcm/falling/" + robot_type_name + "/threshold_gyro_y_front")# \
                                       #+ math.radians(rospy.get_param("ZMPConfig/" + robot_type_name + "/HipPitch", -10))
        rospy.set_param("hcm/threshold_gyro_y_front", self.falling_threshold_front)
        self.falling_threshold_side = rospy.get_param("hcm/falling/" + robot_type_name + "/threshold_gyro_x_side")
        rospy.set_param("hcm/threshold_gyro_x_side", self.falling_threshold_side)
        self.falling_threshold_orientation_front_back = math.radians(rospy.get_param("hcm/falling/" + robot_type_name + "/falling_threshold_orientation_front_back"))
        rospy.set_param("hcm/falling_threshold_orientation_front_back", self.falling_threshold_orientation_front_back)
        self.falling_threshold_orientation_left_right = math.radians(rospy.get_param("hcm/falling/" + robot_type_name + "/falling_threshold_orientation_left_right"))
        rospy.set_param("hcm/falling_threshold_orientation_left_right", self.falling_threshold_orientation_left_right)

        # Grenzwerte an Untergrund anpassen
        self.falling_threshold_front *= self.ground_coefficient
        self.falling_threshold_side *= self.ground_coefficient

        # Neue Werte für die Detectierung des Fallens, anhand der momentanen Lage. Schwellwerte in Grad.
        #self.falling_threshold_orientation_front_back = math.radians(45) 
        #self.falling_threshold_orientation_left_right = math.radians(45)
        

    def update_reconfigurable_values(self, config, level):
        self.dyn_falling_active = config["dyn_falling_active"]
        self.ground_coefficient = config["ground_coefficient"]
        self.falling_threshold_front = config["threshold_gyro_y_front"]
        self.falling_threshold_side = config["threshold_gyro_x_side"]
        self.falling_threshold_orientation_front_back = math.radians(config["falling_threshold_orientation_front_back"])
        self.falling_threshold_orientation_left_right = math.radians(config["falling_threshold_orientation_left_right"])
        

    def check_falling(self, not_much_smoothed_gyro, quaternion):
        """Checks if the robot is currently falling and in which direction. """
        # converting the Quaternion into Euler angles for better understanding
        euler = self.quaternion_to_euler_angle(*quaternion)
        # setting the fall quantification function
        x_fall_quantification = self.calc_fall_quantification(self.falling_threshold_orientation_left_right, self.falling_threshold_front, euler[0], not_much_smoothed_gyro[0])
        y_fall_quantification = self.calc_fall_quantification(self.falling_threshold_orientation_front_back, self.falling_threshold_side, euler[1], not_much_smoothed_gyro[1])

        if x_fall_quantification + y_fall_quantification == 0:
            return None
        
        # compare quantification functions
        if y_fall_quantification > x_fall_quantification:
            # detect the falling direction
            if not_much_smoothed_gyro[1] > 0:
                rospy.loginfo("FALLING TO THE FRONT")
                #TODO remove comments when out off static testing
                return #self.falling_motor_degrees_front
            # detect the falling direction
            else:
                rospy.loginfo("FALLING TO THE BACK")
                return #self.falling_motor_degrees_back
        else:
            # detect the falling direction
            if not_much_smoothed_gyro[0] > 0:
                rospy.loginfo("FALLING TO THE RIGHT")
                return #self.falling_motor_degrees_right
            # detect the falling direction
            else:
                rospy.loginfo("FALLING TO THE LEFT")
                return #self.falling_motor_degrees_left

    def calc_fall_quantification(self, falling_threshold_orientation, falling_threshold_gyro, current_axis_euler, current_axis__gyro):
        # check if you are moving forward or away from the perpendicular position, by comparing the signs.
        if numpy.sign(current_axis_euler) != numpy.sign(current_axis__gyro):
            # calculatiung the orentation skalar for the threshold
            skalar = max((falling_threshold_orientation - abs(current_axis_euler))/falling_threshold_orientation,0)
            # checking if the rotation velocity is lower than the thethreshold
            if falling_threshold_gyro * skalar < abs(current_axis__gyro):
                # returning the fall quantification function
                return abs(current_axis__gyro) * (1-skalar)
        return 0

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

    def quaternion_to_euler_angle(self, x, y, z, w):
        ysqr = y * y
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.atan2(t3, t4)
        
        return X, Y, Z
