# -*- coding:utf-8 -*-
import time

import rospy
from humanoid_league_msgs.msg import MotionState

from bitbots_cm730.srv import SwitchMotorPower

from bitbots_motion.src.fall_checker import FallChecker


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
    """
    We use this class as a singleton to share these public variables over all
    different classes of states and with the state machine.
    """
    def __init__(self):
        self.penalized = False  # paused
        self.record = False

        self.standupflag = False
        self.softstart = False
        self.dieflag = False

        self.last_hardware_update = None
        self.last_request = None
        self.start_up_time = rospy.Time.now()

        self.raw_gyro = (0,0,0)
        self.smooth_gyro = (0,0,0)
        self.not_so_smooth_gyro = (0,0,0)

        self.fall_checker = FallChecker()
        self.animation_client = None

VALUES = Values()

class AbstractState(object):

    def __init__(self):
        # bool, is an animation started, where we want to wait for its finish
        self.animation_started = False
        self.next_state = None

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

    def start_animation(self, anim, follow_state):
        """
        This will NOT wait by itself. You have to check
        VALUES.animation_client.get_result()
        in the evaluate method by yourself.
        :param anim:
        :param follow_state:
        :return:
        """
        self.animation_started = True
        play_animation(anim)
        self.next_state = follow_state


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
        #leave this if we got a hardware response, or after some time
        #todo zeit parameter?
        if VALUES.last_hardware_update is not None or time.time() - VALUES.start_up_time > 1:
            # check if we directly go into a special state, if not, got to get up
            if values.get_record():
                switch_motor_power(True)
                return Record()
            if values.get_softstart():
                switch_motor_power(False)
                values.last_client_update = rospy.Time.now() - 120
                return Softoff()
            switch_motor_power(True)
            if values.get_penalized():
                return PenaltyAnimationIn()
            return GettingUp()

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_STARTUP


class Softoff(AbstractState):

    def entry(self, values):
        switch_motor_power(False)

    def evaluate(self, values):
        if not self.animation_started:
            if VALUES.record:
                switch_motor_power(True)
                # don't directly change state, we wait for animation to finish
                self.start_animation(rospy.get_param("/animations/motion/walkready"), Record())
                return
            if rospy.Time.now() - VALUES.last_request < 10: #todo param
                switch_motor_power(True)
                play_animation(rospy.get_param("/animations/motion/walkready")), Controlable()
                return
            rospy.sleep(0.1)
        else:
            if VALUES.animation_client.get_result():
                return self.next_state

    def exit(self, values):
        pass

    def motion_state(self):
        #we want to look controlable to the outside
        return STATE_CONTROLABLE


class Record(AbstractState):
    def entry(self, values):
        pass

    def evaluate(self, values):
        if not VALUES.record:
            return Controlable()

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_RECORD

class PenaltyAnimationIn(AbstractState):

    def entry(self, values):
        rospy.logwarn("Penalized, sitting down!")
        self.start_animation(rospy.get_param("/animations/motion/penalized"), Penalty())

    def evaluate(self, values):
        #wait for animation started in entry
        if VALUES.animation_client.get_result():
            return self.next_state

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_PENALTY_ANIMANTION

class PenaltyAnimationOut(AbstractState):

    def entry(self, values):
        rospy.logwarn("Not penalized anymore, getting up!")
        self.start_animation(rospy.get_param("/animations/motion/penalized_end"), Controlable())

    def evaluate(self, values):
        # wait for animation started in entry
        if VALUES.animation_client.get_result():
            return self.next_state

    def exit(self, values):
        pass

    def motion_state(self):
        return STATE_PENALTY_ANIMANTION

class Penalty(AbstractState):
    def entry(self, values):
        switch_motor_power(False)
        rospy.loginfo("Im now penalized")

    def evaluate(self, values):
        if VALUES.penalized:
            #still penalized, lets wait a bit
            rospy.sleep(0.05)
            # prohibit soft off
            values.set_last_client_update(rospy.Time.now())
        else:
            return PenaltyAnimationOut()

    def exit(self, values):
        switch_motor_power(True)

    def motion_state(self):
        return STATE_PENALTY


class GettingUp(AbstractState):
    def entry(self, values):
        rospy.logdebug("Getting up!")
        self.start_animation(
            VALUES.fall_checker.check_fallen(VALUES.raw_gyro, VALUES.smooth_gyro),
            Controlable())

    def evaluate(self, values):
        # wait for animation started in entry
        if VALUES.animation_client.get_result():
            # we stood up, but are we now really standing correct
            if VALUES.fall_checker.check_falling(VALUES.not_so_smooth_gyro) is not None:
                #we're falling, directly going to falling
                return Falling()
            elif VALUES.fall_checker.check_fallen(VALUES.raw_gyro, VALUES.smooth_gyro) is None:
                #we're fallen, directly going to fallen
                return Fallen()
            else:
                #everything is fine, head on
                return self.next_state

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


def switch_motor_power(state):
    """ Calling service from CM730 to turn motor power on or off"""
    rospy.wait_for_service("switch_motor_power")
    power_switch = rospy.ServiceProxy("switch_motor_power", SwitchMotorPower)
    try:
        response = power_switch(state)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))

def play_animation(anim_name):
    goal = AnimationActionMsg().anim = anim_name
    VALUES.animation_client.send_goal(goal)

def check_if_animation_finished():
    return VALUES.animation_client.get_result()
