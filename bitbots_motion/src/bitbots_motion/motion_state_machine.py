# -*- coding:utf-8 -*-
import time

import rospy
from bitbots_cm730.srv import SwitchMotorPower
from humanoid_league_msgs.msg import MotionState

from .abstract_state_machine import AbstractState, VALUES
from .abstract_state_machine import AbstractStateMachine

STATE_CONTROLABLE = 0
STATE_FALLING = 1
STATE_FALLEN = 2
STATE_GETTING_UP = 3
STATE_ANIMATION_RUNNING = 4
STATE_STARTUP = 5
STATE_SHUT_DOWN = 6
STATE_PENALTY = 7
STATE_PENALTY_ANIMATION = 8
STATE_RECORD = 9
STATE_WALKING = 10


class MotionStateMachine(AbstractStateMachine):
    def __init__(self, die_flag, standup_flag, soft_off_flag, soft_start, start_test, state_publisher):
        super(MotionStateMachine, self).__init__()
        VALUES.standup_flag = standup_flag
        VALUES.die_flag = die_flag
        VALUES.soft_off_flag = soft_off_flag
        VALUES.soft_start = soft_start
        VALUES.start_test = start_test
        VALUES.last_request = 0
        VALUES.last_hardware_update = 0
        self.error_state = ShutDown()
        self.state_publisher = state_publisher

        self.connections = {Startup: (Controllable, Softoff, GettingUp, Record, PenaltyAnimationIn),
                            Softoff: (Controllable, ShutDown, Record),
                            Record: Controllable,
                            PenaltyAnimationIn: Penalty,
                            Penalty: PenaltyAnimationOut,
                            PenaltyAnimationOut: Controllable,
                            GettingUp: (Falling, Fallen, GettingUpSecond),
                            GettingUpSecond: (Falling, Fallen, Controllable),
                            Controllable: (ShutDownAnimation, Record, PenaltyAnimationIn, Softoff, Falling, Fallen,
                                            AnimationRunning, Walking),
                            Falling: (Fallen, Controllable),
                            Fallen: GettingUp,
                            Walking: (ShutDownAnimation, Record, PenaltyAnimationIn, Softoff, Falling, Fallen,
                                       WalkingStopping, Controllable),
                            WalkingStopping: Controllable,
                            AnimationRunning: Controllable,
                            ShutDownAnimation: ShutDown}

        self.set_state(Startup())

    def publish_state(self, state):
        msg = MotionState()
        msg.state = state.motion_state()
        self.state_publisher.publish(msg)

    def is_penalized(self):
        return self.state.__class__ in (Penalty, PenaltyAnimationIn, PenaltyAnimationOut)

    def is_walking(self):
        return self.state.__class__ in (Walking, WalkingStopping)

    def is_record(self):
        return self.state.__class__ is Record

    def is_shutdown(self):
        return self.state.__class__ is ShutDown


class Startup(AbstractState):
    def entry(self):
        self.start_time = rospy.get_param("/motion/start_time", 10)

    def evaluate(self):
        # leave this if we got a hardware response, or after some time
        # todo zeit parameteriesern?
        if VALUES.last_hardware_update is not 0 or time.time() - VALUES.start_up_time > self.start_time:
            # check if we directly go into a special state, if not, got to get up
            if VALUES.start_test:
                # todo
                pass
                return
            if VALUES.record:
                switch_motor_power(True)
                return Record()
            if VALUES.soft_start:
                switch_motor_power(False)
                # to prohibit getting directly out of softoff
                VALUES.last_client_update = time.time() - 120
                return Softoff()
            switch_motor_power(True)
            if VALUES.penalized:
                return PenaltyAnimationIn()

            # normal states
            if VALUES.standup_flag:
                return GettingUp()
            else:
                return Controllable()
        else:
            rospy.loginfo_throttle(1, "Motion is waiting for data from hardware")

    def exit(self):
        pass

    def motion_state(self):
        return STATE_STARTUP


class Softoff(AbstractState):
    def entry(self):
        switch_motor_power(False)

    def evaluate(self):
        if VALUES.shut_down:
            return ShutDown()
        if not self.animation_started:
            if VALUES.record:
                # todo prohibit sudden movement by getting first one time the current motor values
                switch_motor_power(True)
                self.next_state = Record()
                # don't directly change state, we wait for animation to finish
                self.start_animation(rospy.get_param("/motion/animations/walkready"))
                return
            if time.time() - VALUES.last_request < 10:  # todo param
                # got a new move request
                switch_motor_power(True)
                self.next_state = Controllable()
                self.start_animation(rospy.get_param("/motion/animations/walkready"))
                return
            if VALUES.is_die_time():
                return ShutDown()
            if VALUES.external_animation_requested:
                switch_motor_power(True)
                return Controllable()
            rospy.sleep(0.1)
        else:
            if self.animation_finished():
                return self.next_state

    def exit(self):
        pass

    def motion_state(self):
        # we want to look controlable to the outside
        return STATE_CONTROLABLE


class Record(AbstractState):
    def entry(self):
        pass

    def evaluate(self):
        if not VALUES.record:
            return Controllable()

    def exit(self):
        pass

    def motion_state(self):
        return STATE_RECORD


class PenaltyAnimationIn(AbstractState):
    def entry(self):
        rospy.logwarn("Penalized, sitting down!")
        self.next_state = Penalty()
        self.start_animation(rospy.get_param("/animations/motion/penalized"))

    def evaluate(self):
        # wait for animation started in entry
        if self.animation_finished():
            return self.next_state

    def exit(self):
        pass

    def motion_state(self):
        return STATE_PENALTY_ANIMATION


class PenaltyAnimationOut(AbstractState):
    def entry(self):
        rospy.logwarn("Not penalized anymore, getting up!")
        self.next_state = Controllable()
        self.start_animation(rospy.get_param("/animations/motion/penalized_end"))

    def evaluate(self):
        # wait for animation started in entry
        if VALUES.finished():
            return self.next_state

    def exit(self):
        pass

    def motion_state(self):
        return STATE_PENALTY_ANIMATION


class Penalty(AbstractState):
    def entry(self):
        switch_motor_power(False)
        rospy.loginfo("Im now penalized")

    def evaluate(self):
        if VALUES.penalized:
            # still penalized, lets wait a bit
            rospy.sleep(0.05)
            # prohibit soft off
            VALUES.last_client_update = time.time()
        else:
            return PenaltyAnimationOut()

    def exit(self):
        switch_motor_power(True)

    def motion_state(self):
        return STATE_PENALTY


class GettingUp(AbstractState):
    def entry(self):
        rospy.logdebug("Getting up!")
        self.next_state = GettingUpSecond()
        self.start_animation( #todo this line is just totally wrong
            VALUES.fall_checker.check_fallen(VALUES.raw_gyro, VALUES.smooth_gyro, VALUES.robo_angle))

    def evaluate(self):
        # wait for animation started in entry
        if self.animation_finished():
            # we stood up, but are we now really standing correct?
            if VALUES.is_falling():
                # we're falling, directly going to falling
                return Falling()
            elif VALUES.is_fallen():
                # we're fallen, directly going to fallen
                return Fallen()
            else:
                # everything is fine, head on
                return self.next_state

    def exit(self):
        pass

    def motion_state(self):
        return STATE_GETTING_UP


class GettingUpSecond(AbstractState):
    def entry(self):
        self.next_state = Controllable()
        self.start_animation(rospy.get_param("/motion/animations/walkready"))

    def evaluate(self):
        if self.animation_finished():
            # we stood up, but are we now really standing correct?
            if VALUES.is_falling():
                # we're falling, directly going to falling
                return Falling()
            elif VALUES.is_fallen():
                # we're fallen, directly going to fallen
                return Fallen()
            else:
                # everything is fine, head on
                return self.next_state

    def exit(self):
        pass

    def motion_state(self):
        return STATE_GETTING_UP


class Controllable(AbstractState):
    def entry(self):
        pass

    def evaluate(self):
        if VALUES.shut_down:
            return ShutDownAnimation()
        if VALUES.record:
            return Record()
        if VALUES.penalized:
            return PenaltyAnimationIn()
        if VALUES.is_soft_off_time():
            return Softoff()
        if VALUES.is_die_time():
            return ShutDownAnimation()

        if VALUES.standupflag:
            ## Falling detection
            falling_pose = VALUES.fall_checker.check_falling(VALUES.not_so_smooth_gyro)
            if falling_pose:
                return Falling()
            ### Standing up
            direction_animation = VALUES.fall_checker.check_fallen(VALUES.raw_gyro, VALUES.smooth_gyro,
                                                                   VALUES.robo_angle)
            if direction_animation is not None:
                return Fallen()

        if VALUES.external_animation_play:
            return AnimationRunning()

        if VALUES.walking_active:
            return Walking()

    def exit(self):
        pass

    def motion_state(self):
        return STATE_CONTROLABLE


class Falling(AbstractState):
    def entry(self):

        # go directly in falling pose
        falling_pose = VALUES.fall_checker.get_falling_pose(VALUES.not_so_smooth_gyro)
        if falling_pose is not None:
            self.next_state = Falling()
            self.start_animation(falling_pose)
        else:
            if VALUES.is_fallen():
                # go directly to fallen
                return Fallen()
            else:
                return Controllable()

    def evaluate(self):
        if self.animation_finished():
            return self.next_state

    def exit(self):
        pass

    def motion_state(self):
        return STATE_FALLING


class Fallen(AbstractState):
    def entry(self):
        direction_animation = VALUES.fall_checker.get_falling_pose(VALUES.not_so_smooth_gyro)
        self.next_state = GettingUp()
        self.start_animation(direction_animation)

    def evaluate(self):
        if self.animation_finished():
            return self.next_state

    def exit(self):
        pass

    def motion_state(self):
        return STATE_FALLEN


class Walking(AbstractState):
    def entry(self):
        pass

    def evaluate(self):
        if VALUES.shut_down:
            return ShutDownAnimation()
        if VALUES.record:
            return Record()
        if VALUES.penalized:
            return PenaltyAnimationIn()
        if VALUES.is_soft_off_time():
            return Softoff()
        if VALUES.is_die_time():
            return ShutDownAnimation()
        if VALUES.standupflag:
            ## Falling detection
            if VALUES.is_falling():
                return Falling()
            ### Standing up
            if VALUES.is_fallen():
                return Fallen()

        if VALUES.external_animation_requested:
            return WalkingStopping()

        if not VALUES.walking_active:
            return Controllable()

            # todo request walk positions?

    def exit(self):
        VALUES.walking_active = False

    def motion_state(self):
        return STATE_WALKING


class WalkingStopping(AbstractState):
    def entry(self):
        # todo walking stop
        pass

    def evaluate(self):
        # todo if wakling stopped
        return Controllable()

    def exit(self):
        VALUES.walking_active = False

    def motion_state(self):
        return STATE_WALKING


class AnimationRunning(AbstractState):
    def entry(self):
        # reset flags
        VALUES.external_animation_play = False
        VALUES.external_animation_requested = False

    def evaluate(self):
        # if external animation finished
        if not VALUES.external_animation_finished:
            return Controllable()

    def exit(self):
        pass

    def motion_state(self):
        return STATE_ANIMATION_RUNNING


class ShutDownAnimation(AbstractState):
    def entry(self):
        rospy.loginfo("Motion will shut off")
        VALUES.say("Motion will shut off")
        self.start_animation(rospy.get_param("/animations/shut_down"))

    def evaluate(self):
        if self.animation_finished():
            return ShutDown()

    def exit(self):
        pass

    def motion_state(self):
        return STATE_SHUT_DOWN


class ShutDown(AbstractState):
    def entry(self):
        VALUES.say("Motion says good bye")
        # give humans the chance to hold the robot before turning it off
        rospy.sleep(3)
        switch_motor_power(False)
        rospy.loginfo("Motion is now off, good bye")

    def evaluate(self):
        exit("Motion was shut down")

    def exit(self):
        pass

    def motion_state(self):
        return STATE_SHUT_DOWN


def switch_motor_power(state):
    """ Calling service from CM730 to turn motor power on or off. But only if not using simulator"""
    if rospy.get_param("/simulation_active", False):
        rospy.loginfo("I'm simulating, not switching motorpower to " + state.__str__())
    else:
        # todo set motor ram here if turned on, bc it lost it
        try:
            rospy.wait_for_service("switch_motor_power", timeout=1)
        except rospy.ROSException:
            rospy.logfatal("Can't switch of motorpower, seems like the CM730 is missing.")
            return
        power_switch = rospy.ServiceProxy("switch_motor_power", SwitchMotorPower)
        try:
            response = power_switch(state) #todo do something with respons
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        # wait for motors
        rospy.sleep(1)
