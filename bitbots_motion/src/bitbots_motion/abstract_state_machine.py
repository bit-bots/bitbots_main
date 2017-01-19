# -*- coding:utf-8 -*-
import actionlib
import rospy
import time
from std_msgs.msg import String

from .fall_checker import FallChecker
from bitbots_common.utilCython.pydatavector import PyIntDataVector as IntDataVector
from bitbots_common.utilCython.pydatavector import PyDataVector as DataVector

import bitbots_animation.msg


class Values(object):
    """
    We use this class as a singleton to share these public variables over all
    different classes of states and with the state machine.
    """

    def __init__(self):
        self.penalized = False  # paused
        self.record = False
        self.shut_down = False

        self.standup_flag = False
        self.soft_off_flag = True
        self.soft_start = False
        self.die_flag = False
        self.start_test = False

        self.last_hardware_update = None
        self.last_request = None
        self.start_up_time = time.time()

        self.raw_gyro = IntDataVector(0, 0, 0)
        self.smooth_gyro = IntDataVector(0, 0, 0)
        self.not_so_smooth_gyro = IntDataVector(0, 0, 0)
        self.robo_angle = DataVector(0, 0, 0)

        self.fall_checker = FallChecker()
        # for internal animations
        self.animation_client = actionlib.SimpleActionClient('fibonacci', bitbots_animation.msg.PlayAnimationAction)

        # we want to play an animation, try to become controlable
        self.external_animation_requested = False
        # play now the external animation, go to animation running
        self.external_animation_play = False
        # the animation is finished, go back to controlable
        self.external_animation_finished = False

        # are we walking?
        self.walking_active = False

        self.softoff_time = rospy.get_param("/motion/soft_off_time")
        self.die_time = rospy.get_param("/motion/die_time")

    def is_falling(self):
        falling_pose = self.fall_checker.check_falling(self.not_so_smooth_gyro)
        if falling_pose is not None:
            return True
        return False

    def is_fallen(self):
        direction_animation = self.fall_checker.check_fallen(self.raw_gyro, self.smooth_gyro,
                                                             self.robo_angle)
        if direction_animation is not None:
            return True
        return False

    def is_soft_off_time(self):
        return self.soft_off_flag and time.time() - self.last_hardware_update > self.softoff_time

    def is_die_time(self):
        return self.die_flag and time.time() - self.last_hardware_update > self.die_time

    def animation_finished(self):
        if self.animation_client.get_state() != 9: #the client was started
            return self.animation_client.get_result()
        else:
            rospy.logwarn_throttle(1, "Tried to ask if animation is finished, but no animation was started.")
            return True #no animation was started, so we won't wait for anything

    def say(self, text):
        # todo
        rospy.logwarn("Say not implemented in abstrace state machine: " + text)
        pass


VALUES = Values()


class AbstractState(object):
    def __init__(self):
        # bool, is an animation started, where we want to wait for its finish
        self.animation_started = False
        self.next_state = None

    def entry(self):
        msg = "You schuld overrride entry() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def exit(self):
        """
        You can't change states in the exit method
        :param values:
        :return:
        """
        msg = "You schuld overrride exit() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def evaluate(self):
        msg = "You schuld overrride evaluate() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def motion_state(self):
        msg = "You schuld overrride motion_state() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def start_animation(self, anim):
        """
        This will NOT wait by itself. You have to check
        VALUES.animation_finished()
        in the evaluate method by yourself.
        :param anim:
        :param follow_state:
        :return:
        """
        started = self.play_animation(anim)
        self.animation_started = started

    def play_animation(self, anim_name):
        if anim_name is None or anim_name == "empty":
            rospy.logwarn("Tried to play an animation with an empty name!")
            return False
        try:
            VALUES.animation_client.wait_for_server()
        except rospy.ROSException:
            rospy.logerr(
                "Animation Action Server not running! Motion can not work without animation action server. "
                "Will now wait until server is assailable!")
            VALUES.animation_client.wait_for_server()
        goal = bitbots_animation.msg.PlayAnimationGoal(animation=anim_name)
        VALUES.animation_client.send_goal(goal)
        return True

class AbstractStateMachine(object):
    def __init__(self):
        self.state = None
        self.connections = []
        self.error_state = None
        self.debug_active = rospy.get_param("/debug_active", False)
        if self.debug_active:
            self.debug_publisher = rospy.Publisher("/motion_state_debug", String, queue_size=10)

    def set_state(self, state):
        """
        Switch to another state. Call corresponding exit and entry methods.
        :param state:
        :return:
        """
        try:
            if self.state is not None:
                self.publish_state(state)
                self.state.exit()
            self.state = state
            if self.debug_active:
                # publish name of the current state if debug is active
                self.debug_publisher.publish(self.state.__class__.__name__)
                print("Current state: " + self.state.__class__.__name__)
            entry_switch = self.state.entry()
            if entry_switch is not None:
                # we directly do another state switch
                self.set_state(entry_switch)
        except Exception as exc:
            rospy.logerr(exc)
            self.state = self.error_state
            self.state.entry()

    def evaluate(self):
        """
        Evaluates the current state
        :return:
        """

        switch_state = self.state.evaluate()

        if switch_state is not None:
            if switch_state.__class__ in self.connections[self.state.__class__]:
                self.set_state(switch_state)
            else:
                print(
                    "Not allowed transistion from " + self.state.__class__.__name__ + " to " +
                    switch_state.__class__.__name__)

    def publish_state(self, state):
        msg = "You schuld overrride publish_state() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def get_current_state(self):
        return self.state.motion_state()


