# -*- coding: utf8 -*-
import numpy
import rospy


class FallChecker(object):
    def __init__(self):
        robot_type_name = rospy.get_param("/robot_type_name")
        self.dyn_falling_active = rospy.get_param("/motion/falling/" + robot_type_name + "/dyn_falling_active")
        self.ground_coefficient = rospy.get_param("/motion/falling/" + robot_type_name + "/ground_coefficient")

        # Fallanimation laden
        self.falling_motor_degrees_front = rospy.get_param(
            "/motion/falling/falling_front")
        self.falling_motor_degrees_back = rospy.get_param(
            "/motion/falling/falling_back")
        self.falling_motor_degrees_right = rospy.get_param(
            "/motion/falling/falling_right")
        self.falling_motor_degrees_left = rospy.get_param(
            "/motion/falling/falling_left")

        # Fallerkennungs Grenzwerte laden
        self.falling_threshold_front = rospy.get_param("/motion/falling/" + robot_type_name + "/threshold_gyro_y_front") \
                                       + rospy.get_param("/ZMPConfig/" + robot_type_name + "/HipPitch", 0)
        self.falling_threshold_back = rospy.get_param("/motion/falling/" + robot_type_name + "/threshold_gyro_y_back") \
                                      + rospy.get_param("/ZMPConfig/" + robot_type_name + "/HipPitch", 0)
        self.falling_threshold_right = rospy.get_param(
            "/motion/falling/" + robot_type_name + "/threshold_gyro_x_right")
        self.falling_threshold_left = rospy.get_param("/motion/falling/" + robot_type_name + "/threshold_gyro_x_left")

        # Grenzwerte an Untergrund anpassen
        self.falling_threshold_front *= self.ground_coefficient
        self.falling_threshold_back *= self.ground_coefficient
        self.falling_threshold_right *= self.ground_coefficient
        self.falling_threshold_left *= self.ground_coefficient

    def update_reconfigurable_values(self, config, level):
        self.dyn_falling_active = config["dyn_falling_active"]
        self.ground_coefficient = config["ground_coefficient"]
        self.falling_threshold_front = config["threshold_gyro_y_front"]
        self.falling_threshold_back = config["threshold_gyro_y_back"]
        self.falling_threshold_right = config["threshold_gyro_x_right"]
        self.falling_threshold_left = config["threshold_gyro_x_left"]

    def check_falling(self, not_much_smoothed_gyro):
        """Checks if the robot is currently falling and in which direction. """
        # First decide if we fall more sidewards or more front-back-wards. Then decide if we fall badly enough
        # to do something about it
        if abs(not_much_smoothed_gyro[1]) > abs(not_much_smoothed_gyro[0]):
            falling_pose = self.check_falling_front_back(not_much_smoothed_gyro)
        else:
            falling_pose = self.check_falling_sideways(not_much_smoothed_gyro)
        return falling_pose

    def check_falling_front_back(self, not_much_smoothed_gyro):
        # Am I falling backwards
        if self.falling_threshold_back < not_much_smoothed_gyro[1]:
            rospy.logdebug("FALLING BACKWARDS ")
            return self.falling_motor_degrees_back
        # Am I falling to the front
        if not_much_smoothed_gyro[1] < self.falling_threshold_front:
            rospy.logdebug("FALLING TO THE FRONT")
            return self.falling_motor_degrees_front

    def check_falling_sideways(self, not_much_smoothed_gyro):
        # Am I falling to the right
        if not_much_smoothed_gyro[0] < self.falling_threshold_right:
            rospy.logdebug("FALLING TO THE RIGHT")
            return self.falling_motor_degrees_right
        # Am I falling to the left
        if self.falling_threshold_left < not_much_smoothed_gyro[0]:
            rospy.logdebug("FALLING TO THE LEFT")
            return self.falling_motor_degrees_left

    def check_fallen(self, raw_gyro, smooth_gyro, robo_angle):
        # todo where the fuck comes robo_angle from and what is this magic
        """Check if the robot has fallen and is lying on the floor. Returns animation to play, if necessary."""
        if numpy.linalg.norm(raw_gyro) < 5 and numpy.linalg.norm(smooth_gyro) < 5 and robo_angle[1] > 80:  ###gyro
            rospy.logdebug("Lying on belly, should stand up")
            return rospy.get_param("/motion/animations/front-up")

        if numpy.linalg.norm(raw_gyro) < 5 and numpy.linalg.norm(smooth_gyro) < 5 and robo_angle[1] < -60:  ###gyro
            rospy.logdebug("Lying on my back, should stand up!")
            return rospy.get_param("/motion/animations/bottom-up")

        if numpy.linalg.norm(raw_gyro) < 5 and numpy.linalg.norm(smooth_gyro) < 5 and abs(robo_angle[0]) > 60:
            rospy.logdebug("Lying on the side, should stand up. Trying to stand up from front. Is that right?")
            return rospy.get_param("/motion/animations/front-up")

        if numpy.linalg.norm(raw_gyro) < 3 and numpy.linalg.norm(smooth_gyro) < 3:
            rospy.logdebug("I think I am still kind of upright, trying to go to walkready")
            return rospy.get_param("/motion/animations/walkready")
