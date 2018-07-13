import rospy
import numpy
from geometry_msgs.msg import Twist

from .fall_checker import FallChecker

import time


class Blackboard(object):
    """
    We use this class as a singleton to share these public variables over all
    different classes of states, the state machine and the hcm node.
    """

    def __init__(self):
        self.speak_publisher = None

        self.penalized = False  # paused
        self.record = False  # record UI running
        self.shut_down = False  # do we want to shut down

        self.stand_up_active = True # should the robot stand up        
        self.motors_on = False  # are the motors on
        self.motors_on_start = False  # should the motors be turned on at start
        self.timed_motor_off = False  # deactivate motors after some time without commands

        self.last_motor_update = None  # time of last update from hardware
        self.last_motor_command = None  # last request on doing something
        self.start_up_time = 0 # time at start of HCM

        self.raw_gyro = numpy.array([0, 0, 0])
        self.smooth_gyro = numpy.array([0, 0, 0])
        self.not_so_smooth_gyro = numpy.array([0, 0, 0])
        self.smooth_accel = numpy.array([0, 0, 0])
        self.quaternion  = numpy.array([0,0,0,0.21])

        self.fall_checker = FallChecker()
        # for internal animations
        self.animation_client = None
        self.hcm_animation_playing = False
        self.hcm_animation_finished = False

        # we want to play an animation, try to become controllable
        self.external_animation_requested = False
        # playing now the external animation, go to animation running
        self.external_animation_playing = False
        # the animation is finished, go back to controllable
        self.external_animation_finished = False

        # are we walking?
        self.walking_active = False

        # used to stop the walking
        # TODO evaluate if necessary
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.motor_off_time = rospy.get_param("hcm/motor_off_time")        
        # TODO disable Simulation
        self.simulation_active = rospy.get_param("simulation_active")


    def is_falling(self):
        falling_pose = self.fall_checker.check_falling(self.not_so_smooth_gyro, self.quaternion)
        if falling_pose is not None:
            return True
        return False

    def get_falling_pose(self):
        return self.fall_checker.check_falling(self.not_so_smooth_gyro, self.quaternion)

    def is_fallen(self):
        return self.fall_checker.check_fallen(self.smooth_accel)

    def is_soft_off_time(self):
        if self.simulation_active:
            return False
        if self.last_hardware_update is not None:
            return self.die_flag and rospy.get_time() - self.last_hardware_update > self.softoff_time
        else:
            return self.die_flag and rospy.get_time() - self.start_up_time > self.softoff_time

    def is_die_time(self):
        return False
        if self.simulation_active:
            return False
        if self.last_hardware_update is not None:
            return self.die_flag and rospy.get_time() - self.last_hardware_update > self.die_time
        else:
            return self.die_flag and rospy.get_time() - self.start_up_time > self.die_time


BLACKBOARD = Blackboard()
