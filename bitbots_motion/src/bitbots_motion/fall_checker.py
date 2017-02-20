# -*- coding: utf8 -*-
import numpy
import rospy


class FallChecker(object):
    def __init__(self):
        # will be set by dynamic reconfigure
        robot_type_name = rospy.get_param("/robot_type_name")

        # Fallanimation laden
        self.falling_motor_degrees_front = rospy.get_param(
            "/motion/falling/falling_front")
        self.falling_motor_degrees_back = rospy.get_param(
            "/motion/falling/falling_back")
        self.falling_motor_degrees_right = rospy.get_param(
            "/motion/falling/falling_right")
        self.falling_motor_degrees_left = rospy.get_param(
            "/motion/falling/falling_left")

        # load config values depending on robot type and lode them into param server to set
        # start values for dynamic reconfigure
        # There are no default values set in dynamic reconfigure, in order to make it dependable on the robot
        self.dyn_falling_active = rospy.get_param("/motion/falling/" + robot_type_name + "/dyn_falling_active")
        rospy.set_param("/motion/dyn_falling_active", self.dyn_falling_active)
        self.ground_coefficient = rospy.get_param("/motion/falling/" + robot_type_name + "/ground_coefficient")
        rospy.set_param("/motion/ground_coefficient", self.ground_coefficient)

        if not rospy.has_param("/ZMPConfig/" + robot_type_name + "/HipPitch"):
            rospy.logwarn("HipPitch offset from walking was not found on parameter server, will use 0.")
        self.falling_threshold_front = rospy.get_param("/motion/falling/" + robot_type_name + "/threshold_gyro_y_front") \
                                       + numpy.math.radians(rospy.get_param("/ZMPConfig/" + robot_type_name + "/HipPitch", 0))
        rospy.set_param("/motion/threshold_gyro_y_front", self.falling_threshold_front)
        self.falling_threshold_back = rospy.get_param("/motion/falling/" + robot_type_name + "/threshold_gyro_y_back") \
                                      + numpy.math.radians(rospy.get_param("/ZMPConfig/" + robot_type_name + "/HipPitch", 0))
        rospy.set_param("/motion/threshold_gyro_y_back", self.falling_threshold_back)
        self.falling_threshold_right = rospy.get_param("/motion/falling/" + robot_type_name + "/threshold_gyro_x_right")
        rospy.set_param("/motion/threshold_gyro_x_right", self.falling_threshold_right)
        self.falling_threshold_left = rospy.get_param("/motion/falling/" + robot_type_name + "/threshold_gyro_x_left")
        rospy.set_param("/motion/threshold_gyro_x_left", self.falling_threshold_left)

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
        return None

    def check_falling_sideways(self, not_much_smoothed_gyro):
        # Am I falling to the right
        if not_much_smoothed_gyro[0] < self.falling_threshold_right:
            rospy.logdebug("FALLING TO THE RIGHT")
            return self.falling_motor_degrees_right
        # Am I falling to the left
        if self.falling_threshold_left < not_much_smoothed_gyro[0]:
            rospy.logdebug("FALLING TO THE LEFT")
            return self.falling_motor_degrees_left
        return None

    def check_fallen(self, smooth_accel):
        """Check if the robot has fallen and is lying on the floor. Returns animation to play, if necessary."""
        if smooth_accel[1] > 6:  ###gyro
            rospy.logdebug("Lying on belly, should stand up")
            return rospy.get_param("/motion/animations/front-up")

        if smooth_accel[1] < -6:  ###gyro
            rospy.logdebug("Lying on my back, should stand up!")
            return rospy.get_param("/motion/animations/bottom-up")

        if abs(smooth_accel[0]) > 6:
            rospy.logdebug("Lying on the side, should stand up. Trying to stand up from front. Is that right?")
            return rospy.get_param("/motion/animations/front-up")

        return None
