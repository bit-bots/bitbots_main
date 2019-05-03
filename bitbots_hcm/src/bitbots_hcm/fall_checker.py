# -*- coding: utf8 -*-
import math
import numpy
import tf
from sensor_msgs.msg import Imu
import rospy

# todo hip pitch offset

class FallChecker(object):
    def __init__(self):

        # will be set by dynamic reconfigure
        robot_type_name = rospy.get_param("robot_type_name", "wolfgang")

        # load config values depending on robot type and lode them into param server to set
        # start values for dynamic reconfigure
        # There are no default vaules set in dynamic reconfigure, in order to make it dependable on the robot
        self.dyn_falling_active = rospy.get_param("hcm/falling/" + robot_type_name + "/dyn_falling_active")
        rospy.set_param("hcm/dyn_falling_active", self.dyn_falling_active)
        self.ground_coefficient = rospy.get_param("hcm/falling/" + robot_type_name + "/ground_coefficient")
        rospy.set_param("hcm/ground_coefficient", self.ground_coefficient)

        if not rospy.has_param("ZMPConfig/" + robot_type_name + "/HipPitch"):
            rospy.logwarn("HipPitch offset from walking was not found on parameter server, will use 0.")
        self.falling_threshold_front = rospy.get_param("hcm/threshold_gyro_y_front")
        # rospy.set_param("hcm/threshold_gyro_y_front", self.falling_threshold_front)
        self.falling_threshold_side = rospy.get_param("hcm/threshold_gyro_x_side")
        # rospy.set_param("hcm/threshold_gyro_x_side", self.falling_threshold_side)
        self.falling_threshold_orientation_front_back = math.radians(rospy.get_param("hcm/falling_threshold_orientation_front_back"))
        # rospy.set_param("hcm/falling_threshold_orientation_front_back", self.falling_threshold_orientation_front_back)
        self.falling_threshold_orientation_left_right = math.radians(rospy.get_param("hcm/falling_threshold_orientation_left_right"))
        # rospy.set_param("hcm/falling_threshold_orientation_left_right", self.falling_threshold_orientation_left_right)

        # Grenzwerte an Untergrund anpassen
        self.falling_threshold_front *= self.ground_coefficient
        self.falling_threshold_side *= self.ground_coefficient

        self.FRONT = "FRONT"
        self.BACK = "BACK"
        self.LEFT = "LEFT"
        self.RIGHT = "RIGHT"
        self.SIDE = "SIDE"
        

    def update_reconfigurable_values(self, config, level):
        # Dynamic Reconfigure
        self.falling_threshold_front = config["threshold_gyro_y_front"]
        self.falling_threshold_side = config["threshold_gyro_x_side"]
        self.falling_threshold_orientation_front_back = math.radians(config["falling_threshold_orientation_front_back"])
        self.falling_threshold_orientation_left_right = math.radians(config["falling_threshold_orientation_left_right"])
        return config

    def check_falling(self, not_much_smoothed_gyro, quaternion):
        """Checks if the robot is currently falling and in which direction. """
        # Checks if robot is still
        if all(abs(n) < 0.1 for n in not_much_smoothed_gyro):
            return

        # converting the Quaternion into Euler angles for better understanding
        euler = tf.transformations.euler_from_quaternion(quaternion)
        if self.falling_threshold_front == 0 or self.falling_threshold_side == 0 or self.falling_threshold_orientation_front_back == 0 or self.falling_threshold_orientation_left_right == 0: 
            return
        # setting the fall quantification function
        x_fall_quantification = self.calc_fall_quantification(self.falling_threshold_orientation_left_right, self.falling_threshold_front, euler[0], not_much_smoothed_gyro[0])
        y_fall_quantification = self.calc_fall_quantification(self.falling_threshold_orientation_front_back, self.falling_threshold_side, euler[1], not_much_smoothed_gyro[1])

        if x_fall_quantification + y_fall_quantification == 0:
            return None
        
        # compare quantification functions
        if y_fall_quantification > x_fall_quantification:
            # detect the falling direction
            if not_much_smoothed_gyro[1] > 0:
                rospy.loginfo("FALLING TO THE BACK")
                return self.BACK
            # detect the falling direction
            else:
                rospy.loginfo("FALLING TO THE FRONT")
                return self.FRONT
        else:
            # detect the falling direction
            if not_much_smoothed_gyro[0] > 0:
                rospy.loginfo("FALLING TO THE LEFT")
                return self.LEFT
            # detect the falling direction
            else:
                rospy.loginfo("FALLING TO THE RIGHT")
                return self.RIGHT
        return None

    def calc_fall_quantification(self, falling_threshold_orientation, falling_threshold_gyro, current_axis_euler, current_axis__gyro):
        # check if you are moving forward or away from the perpendicular position, by comparing the signs.
        if numpy.sign(current_axis_euler) == numpy.sign(current_axis__gyro):
            # calculatiung the orentation skalar for the threshold
            skalar = max((falling_threshold_orientation - abs(current_axis_euler))/falling_threshold_orientation,0)
            # checking if the rotation velocity is lower than the thethreshold
            if falling_threshold_gyro * skalar < abs(current_axis__gyro):
                # returning the fall quantification function
                return abs(current_axis__gyro) * (1-skalar)
        return 0

    def check_fallen(self, smooth_accel, not_much_smoothed_gyro):
        """Check if the robot has fallen and is lying on the floor. Returns animation to play, if necessary."""
        # Checks if the robot is still moving.
        if any(abs(n) >= 0.1 for n in not_much_smoothed_gyro):
            return None

        # Decides which side is facing downwards.
        if smooth_accel[0] > 7:
            rospy.loginfo("FALLEN TO THE FRONT")
            return self.FRONT

        if smooth_accel[0] < -7:
            rospy.loginfo("FALLEN TO THE BACK")
            return self.BACK

        if smooth_accel[1] < -7:
            rospy.loginfo("FALLEN TO THE RIGHT")
            return self.RIGHT

        if smooth_accel[1] > 7:
            rospy.loginfo("FALLEN TO THE LEFT")
            return self.LEFT

        # If no side is facing downwards, the robot is not fallen yet.
        return None