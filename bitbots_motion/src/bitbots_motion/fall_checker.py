# -*- coding: utf8 -*-
import rospy


class FallChecker(object):
    def __init__(self):
        robot_type_name = rospy.get_param("/RobotTypeName")
        self.dyn_falling_active = rospy.get_param("/motion/falling/" + robot_type_name + "/dyn_falling_active")
        self.ground_coefficient = rospy.get_param("/motion/falling/" + robot_type_name + "/ground_coefficient")

        # Fallanimation laden
        self.falling_motor_degrees_front = rospy.get_param(
            "/motion/falling/" + robot_type_name + "/falling_motor_degrees_front")
        self.falling_motor_degrees_back = rospy.get_param(
            "/motion/falling/" + robot_type_name + "/falling_motor_degrees_back")
        self.falling_motor_degrees_right = rospy.get_param(
            "/motion/falling/" + robot_type_name + "/falling_motor_degrees_right")
        self.falling_motor_degrees_left = rospy.get_param(
            "/motion/falling/" + robot_type_name + "/falling_motor_degrees_left")

        # Fallerkennungs Grenzwerte laden
        self.falling_threshold_front = rospy.get_param("/motion/falling/" + robot_type_name + "/threshold_gyro_y_front") \
                                       + rospy.get_param("/ZMPConfig/" + robot_type_name + "/HipPitch")
        self.falling_threshold_back = rospy.get_param("/motion/falling/" + robot_type_name + "/threshold_gyro_y_back") \
                                      + rospy.get_param("/ZMPConfig/" + robot_type_name + "/HipPitch")
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

    def get_falling_pose(self, not_so_smooth_gyro):
        # todo doe something with this
        # todo propably just change it to returning an animation(depending on side)
        if self.dyn_falling_active:
            for i in range(1, 20):
                a = falling_motor_degrees[i - 1]
                goal_pose.get_joint_by_cid(i).goal = a
            pass
        else:
            goal_pose.set_active(False)

    def check_falling(self, not_much_smoothed_gyro):
        """Checks if the robot is currently falling and in which direction. """

        # First decide if we fall more sidewards or more front-back-wards. Then decide if we fall badly enough
        # to do something about it
        if abs(not_much_smoothed_gyro.get_y()) > abs(not_much_smoothed_gyro.get_x()):
            falling_pose = self.check_falling_front_back(not_much_smoothed_gyro)
        else:
            falling_pose = self.check_falling_sideways(not_much_smoothed_gyro)
        return falling_pose

    def check_falling_front_back(self, not_much_smoothed_gyro):
        # Am I falling backwards
        if self.falling_threshold_back < not_much_smoothed_gyro.get_y():
            rospy.logdebug("FALLING BACKWARDS ")
            return self.falling_motor_degrees_back
        # Am I falling to the front
        if not_much_smoothed_gyro.get_y() < self.falling_threshold_front:
            rospy.logdebug("FALLING TO THE FRONT")
            return self.falling_motor_degrees_front

    def check_falling_sideways(self, not_much_smoothed_gyro):
        # Am I falling to the right
        if not_much_smoothed_gyro.get_x() < self.falling_threshold_right:
            rospy.logdebug("FALLING TO THE RIGHT")
            return self.falling_motor_degrees_right
        # Am I falling to the left
        if self.falling_threshold_left < not_much_smoothed_gyro.get_x():
            rospy.logdebug("FALLING TO THE LEFT")
            return self.falling_motor_degrees_left

    def check_fallen(self, raw_gyro, smooth_gyro):
        # todo where the fuck comes robo_angle from and what is this magic
        """Check if the robot has fallen and is lying on the floor. Returns animation to play, if necessary."""
        if raw_gyro.norm() < 5 and smooth_gyro.norm() < 5 and robo_angle.y > 80:  ###gyro
            rospy.logdebug("Lying on belly, should stand up")
            return rospy.get_param("/motion/animations/front-up")

        if raw_gyro.norm() < 5 and smooth_gyro.norm() < 5 and robo_angle.y < -60:  ###gyro
            rospy.logdebug("Lying on my back, should stand up!")
            return rospy.get_param("/motion/animations/bottom-up")

        if raw_gyro.norm() < 5 and smooth_gyro.norm() < 5 and abs(robo_angle.x) > 60:
            rospy.logdebug("Lying on the side, should stand up. Trying to stand up from front. Is that right?")
            return rospy.get_param("/motion/animations/front-up")

        if raw_gyro.norm() < 3 and smooth_gyro.norm() < 3:
            rospy.logdebug("I think I am still kind of upright, trying to go to walkready")
            return rospy.get_param("/motion/animations/walkready")
