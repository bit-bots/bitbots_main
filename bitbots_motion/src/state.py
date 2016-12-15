# -*- coding:utf-8 -*-
import time

import rospy
from humanoid_league_msgs.msg import MotionState

from bitbots_cm730.srv import SwitchMotorPower

from bitbots_motion.src.standuphandler import StandupHandler


STATE_CONTROLABLE = 0
STATE_FALLING = 1
STATE_FALLEN = 2
STATE_GETTING_UP = 3
STATE_ANIMATION_RUNNING = 4
STATE_STARTUP = 5
STATE_SHUT_DOWN = 6
STATE_PENALTY = 7
STATE_PENALTY_ANIMANTION = 8
STATE_RECORD = 9
STATE_WALKING = 10

class Values(object):
    def __init__(self):
        self.penalized = False  # paused
        self.record = False

        self.standupflag = False
        self.softstart = False
        self.dieflag = False

        self.last_hardware_update = 0
        self.last_request = 0

        self.stand_up_handler = StandupHandler()

VALUES = Values()

class AbstractState(object):
    def entry(self, values):
        msg = "You schuld overrride entry() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def exit(self, values):
        msg = "You schuld overrride exit() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def evaluate(self, values):
        msg = "You schuld overrride evaluate() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def motion_state(self):
        msg = "You schuld overrride motion_state() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)


class AbstractStateMachine(object):
    def __init__(self):
        self.state = AbstractState()
        self.connections = []
        self.error_state = None

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
            self.state.entry()
        except:
            self.set_state(self.error_state)

    def evaluate(self):
        """
        Evaluates the current state
        :return:
        """
        switch_state = None        self.values = Values()

        try:
            switch_state = self.state.evaluate()
        except:
            self.set_state(self.error_state)

        if switch_state is not None:
            if (self.state.__class__, switch_state.__class__) in self.connections:
                self.set_state(switch_state)
            else:
                print("not allowed")

    def publish_state(self, state):
        msg = "You schuld overrride publish_state() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def get_current_state(self):
        return self.state.motion_state()

class MotionStateMachine(AbstractStateMachine):
    def __init__(self, standupflag, state_publiser):
        super().__init__()
        VALUES.standupflag = standupflag
        self.error_state = ShutDown()
        self.set_state(StartupState())
        self.state_publisher = state_publiser

    def publish_state(self, state):
        msg = MotionState()
        msg.state = state.motion_state()
        self.state_publisher.publish(msg)

class StartupState(AbstractState):
    def entry(self, values):
        pass

    def evaluate(self, values):
        if values.get_last_motor_update is not None or time.time() - values.get_startup_time() > 1:
            if values.get_record():
                switch_motor_power(True)
                return Record()
            if values.get_softstart():
                switch_motor_power(False)
                values.last_client_update = rospy.Time.now() - 120
                return Softoff()
            switch_motor_power(True)
            if values.get_penalized():
                return Penalty()
            return GettingUp()

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_STARTUP


class Softoff(AbstractState):
    def entry(self, values):
        switch_motor_power(False)

    def evaluate(self, values):
        if VALUES.record:
            return Record()
        if new_move_request:
            return Controlable()
        rospy.sleep(0.1)

    def exit(self, values):
        switch_motor_power(True)
        self.animate(
            rospy.get_param("/animations/motion/walkready")

    def motion_state(self):
        #we want to look controlable to the outside
        return STATE_CONTROLABLE


class Record(AbstractState):
    def entry(self, values):
        pass

    def evaluate(self, values):
        if not values.get_record:
            return Controlable()

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_RECORD

class PenaltyAnimationIn(AbstractState):
    def __init__(self):
        self.animation_finished = False

    def entry(self, values):
        rospy.logwarn("Penalized, sitting down!")
        self.animate(
            rospy.get_param("/animations/motion/penalized"))

    def evaluate(self, values):
        if self.animation_finished:
            return Penalty()

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_PENALTY_ANIMANTION

class PenaltyAnimationIn(AbstractState):
    def __init__(self):
        self.animation_finished = False

    def entry(self, values):
        rospy.logwarn("Penalized, sitting down!")
        self.animate(
            rospy.get_param("/animations/motion/penalized"))

    def evaluate(self, values):
        if self.animation_finished:
            return Penalty()

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_PENALTY_ANIMANTION

class Penalty(AbstractState):
    def entry(self, values):
        switch_motor_power(False)
        rospy.loginfo("Im now penalized")

    def evaluate(self, values):
        rospy.sleep(0.05)
        values.set_last_client_update(rospy.Time.now())

    def exit(self, values):
        switch_motor_power(True)

    def motion_state(self):
        return STATE_PENALTY


class GettingUp(AbstractState):
    def entry(self, values):
        pass

    def evaluate(self, values):
        pass

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_GETTING_UP

class Controlable(AbstractState):
    def entry(self, values):
        pass

    def evaluate(self, values):
        if values.get_standup_flag():
            ## Falling detection
            falling_pose = values.get_stand_up_handler().check_falling(values.not_much_smoothed_gyro)
            if falling_pose:
                values.get_stand_up_handler().set_falling_pose(falling_pose, goal_pose)
                return Falling()
            ### Standing up
            direction_animation = values.stand_up_handler.check_fallen(values.gyro, values.smooth_gyro,
                                                                     values.robo_angle)
            if direction_animation is not None:
                # todo run direction animaiton
                return Fallen()

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_CONTROLABLE

class Falling(AbstractState):
    def entry(self, values):
        ### Standing up
        direction_animation = values.stand_up_handler.check_fallen(values.gyro, values.smooth_gyro,
                                                                   values.robo_angle)
        if direction_animation is not None:
            # todo run direction animaiton
            return Fallen()

    def evaluate(self, values):
        pass

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_FALLING


class Fallen(AbstractState):
    def entry(self, values):
        # directly starting to get up. Sending STATE_FALLEN before is still important, e.g. localisation
        return GettingUp()

    def evaluate(self, values):
        pass

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_FALLEN


class Walking(AbstractState):
    def entry(self, values):
        pass

    def evaluate(self, values):
        pass

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_WALKING


class AnimationRunning(AbstractState):
    def entry(self, values):
        pass

    def evaluate(self, values):
        pass

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_ANIMATION_RUNNING


class ShutDown(AbstractState):
    def entry(self, values):
        pass

    def evaluate(self, values):
        pass

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_SHUT_DOWN


def switch_motor_power(self, state):
    """ Calling service from CM730 to turn motor power on or off"""
    rospy.wait_for_service("switch_motor_power")
    power_switch = rospy.ServiceProxy("switch_motor_power", SwitchMotorPower)
    try:
        response = power_switch(state)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
