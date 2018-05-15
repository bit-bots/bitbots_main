# -*- coding:utf-8 -*-
import time

import rospy
from bitbots_cm730.srv import SwitchMotorPower
from geometry_msgs.msg import Twist
from humanoid_league_msgs.msg import RobotControlState

from .abstract_state_machine import AbstractState, VALUES
from .abstract_state_machine import AbstractStateMachine
from humanoid_league_speaker.speaker import speak
from humanoid_league_msgs.msg import Speak

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


class HcmStateMachine(AbstractStateMachine):
    def __init__(self, die_flag, standup_flag, soft_off_flag, soft_start, start_test, state_publisher):
        super(HcmStateMachine, self).__init__()
        VALUES.standup_flag = standup_flag
        VALUES.die_flag = die_flag
        VALUES.soft_off_flag = soft_off_flag
        VALUES.soft_start = soft_start
        VALUES.start_test = start_test
        VALUES.last_request = 0
        VALUES.last_hardware_update = None

        self.error_state = ShutDown()
        self.state_publisher = state_publisher

        self.connections = rospy.get_param("hcm_state_machine")

        self.set_state(Startup())

    def publish_state(self, state):
        msg = RobotControlState()
        msg.state = state.hcm_state()
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
        self.start_time_limit = rospy.get_param("hcm/start_time", 10)

    def evaluate(self):
        if not self.animation_started:
            # leave this if we got a hardware response, or after some time
            if VALUES.last_hardware_update is not None or rospy.get_time() - VALUES.start_up_time > self.start_time_limit:
                # check if we directly go into a special state, if not, got to get up
                if VALUES.start_test:
                    pass
                    return
                if VALUES.record:
                    switch_motor_power(True)
                    return Record()
                if VALUES.soft_start:
                    switch_motor_power(False)
                    # to prohibit getting directly out of softoff
                    VALUES.last_client_update = rospy.get_time() - 120
                    return Softoff()
                switch_motor_power(True)
                if VALUES.penalized:
                    return PenaltyAnimationIn()

                # normal states
                if VALUES.standup_flag:
                    self.next_state = GettingUp()
                else:
                    self.next_state = Controllable()
                # play init
                self.start_animation(rospy.get_param("hcm/animations/init"))
            else:
                rospy.loginfo_throttle(1, "Motion is waiting for data from hardware")
        else:
            if self.animation_finished():
                return self.next_state

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_STARTUP

    def shutdown(self):
        return ShutDown()


class Softoff(AbstractState):
    def entry(self):
        switch_motor_power(False)

    def evaluate(self):
        if not self.animation_started:
            if VALUES.record:
                switch_motor_power(True)
                self.next_state = Record()
                # don't directly change state, we wait for animation to finish
                self.start_animation(rospy.get_param("hcm/animations/walkready"))
                return
            if rospy.get_time() - VALUES.last_request < 10:
                # got a new move request
                switch_motor_power(True)
                self.next_state = Controllable()
                self.start_animation(rospy.get_param("hcm/animations/walkready"))
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

    def hcm_state(self):
        # we want to look controlable to the outside
        return STATE_CONTROLABLE

    def shutdown(self):
        return ShutDown()


class Record(AbstractState):
    def entry(self):
        pass

    def evaluate(self):
        if not VALUES.record:
            return Controllable()

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_RECORD

    def shutdown(self):
        return ShutDownAnimation()


class PenaltyAnimationIn(AbstractState):
    def entry(self):
        rospy.logwarn("Penalized, sitting down!")
        self.next_state = Penalty()
        self.start_animation(rospy.get_param("hcm/animations/penalized"))

    def evaluate(self):
        # wait for animation started in entry
        if self.animation_finished():
            return self.next_state

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_PENALTY_ANIMATION

    def shutdown(self):
        return ShutDownAnimation()


class PenaltyAnimationOut(AbstractState):
    def entry(self):
        rospy.logwarn("Not penalized anymore, getting up!")
        self.next_state = Controllable()
        self.start_animation(rospy.get_param("hcm/animations/penalized_end"))

    def evaluate(self):
        # wait for animation started in entry
        if self.animation_finished():
            return self.next_state

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_PENALTY_ANIMATION

    def shutdown(self):
        return ShutDownAnimation()


class Penalty(AbstractState):
    def entry(self):
        switch_motor_power(False)
        rospy.loginfo("Im now penalized")

    def evaluate(self):
        if VALUES.penalized:
            # still penalized, lets wait a bit
            rospy.sleep(0.05)
            # prohibit soft off
            VALUES.last_client_update = rospy.get_time()
        else:
            return PenaltyAnimationOut()

    def exit(self):
        switch_motor_power(True)

    def hcm_state(self):
        return STATE_PENALTY

    def shutdown(self):
        return ShutDown()


class GettingUp(AbstractState):
    """This state starts the getting up procedure. """

    def entry(self):
        # normally we should got to getting up second after this
        self.next_state = GettingUpSecond()
        # but lets check if we actually have to stand up
        fallen = VALUES.fall_checker.check_fallen(VALUES.smooth_accel)
        if fallen is not None:
            self.start_animation(fallen)
            pass
        else:
            self.next_state = Controllable()
            self.start_animation(rospy.get_param("hcm/animations/walkready"))

    def evaluate(self):
        # wait for animation started in entry
        if self.animation_finished():
            # head on
            return self.next_state

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_GETTING_UP

    def shutdown(self):
        return ShutDownAnimation()


class GettingUpSecond(AbstractState):
    """This state plays a second animation for getting up."""

    def entry(self):
        self.next_state = Controllable()
        self.start_animation(rospy.get_param("hcm/animations/walkready"))

    def evaluate(self):
        if self.animation_finished():
            # head on
            return self.next_state

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_GETTING_UP

    def shutdown(self):
        return ShutDownAnimation()


class Controllable(AbstractState):
    def entry(self):
        pass

    def evaluate(self):
        if VALUES.record:
            return Record()
        if VALUES.penalized:
            return PenaltyAnimationIn()
        if VALUES.is_soft_off_time():
            return Softoff()
        if VALUES.is_die_time():
            rospy.logwarn("die time")
            return ShutDownAnimation()

        if VALUES.standup_flag:
            ## Falling detection
            if VALUES.is_falling():
                return Falling()
            ### Standing up
            direction_animation = VALUES.fall_checker.check_fallen(VALUES.smooth_accel)
            if direction_animation is not None:
                return Fallen()

        if VALUES.external_animation_playing:
            return AnimationRunning()

        if VALUES.walking_active:
            return Walking()

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_CONTROLABLE

    def shutdown(self):
        return ShutDownAnimation()


class Falling(AbstractState):
    def entry(self):
        self.wait_time = rospy.get_time()
        # go directly in falling pose
        falling_pose = VALUES.get_falling_pose()
        if falling_pose is not None:
            # we're falling, stay in falling
            self.next_state = Falling()
            self.start_animation(falling_pose)
        else:
            # we're not falling anymore
            if VALUES.is_fallen():
                # go directly to fallen
                return Fallen()
            else:
                self.next_state = Controllable()
                self.start_animation(rospy.get_param("hcm/animations/walkready"))

    def evaluate(self):
        # we wait a moment before going to the next state
        if rospy.get_time() - self.wait_time > 3 and self.animation_finished():
            return self.next_state

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_FALLING

    def shutdown(self):
        return ShutDown()


class Fallen(AbstractState):
    def entry(self):
        direction_animation = VALUES.is_fallen()
        if direction_animation is None:
            # we don't have to stand up, go directly to controllable
            return Controllable()
        else:
            # do corresponding animation
            self.next_state = GettingUp()
            self.start_animation(direction_animation)

    def evaluate(self):
        if self.animation_finished():
            return self.next_state

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_FALLEN

    def shutdown(self):
        return ShutDown()


class Walking(AbstractState):
    def entry(self):
        pass

    def evaluate(self):
        if VALUES.record:
            return Record()
        if VALUES.penalized:
            return PenaltyAnimationIn()
        if VALUES.is_soft_off_time():
            return Softoff()
        if VALUES.is_die_time():
            return ShutDownAnimation()
        if VALUES.standup_flag:
            # Falling detection
            if VALUES.is_falling():
                return Falling()
            # Standing up
            if VALUES.is_fallen():
                return Fallen()

        if VALUES.external_animation_requested:
            return WalkingStopping()

        if not VALUES.walking_active:
            return Controllable()

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_WALKING

    def shutdown(self):
        return ShutDownAnimation()


class WalkingStopping(AbstractState):
    def entry(self):
        # publish 0 twist message to stop the walking
        twist = Twist()
        VALUES.cmd_vel_pub.publish()

    def evaluate(self):
        # wait till the walking stopped and go to controlable
        if not VALUES.walking_active:
            return Controllable()

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_WALKING

    def shutdown(self):
        return ShutDownAnimation()


class AnimationRunning(AbstractState):
    def entry(self):
        # reset flags
        VALUES.external_animation_playing = False
        VALUES.external_animation_requested = False

    def evaluate(self):
        # if external animation finished
        if not VALUES.external_animation_finished:
            return Controllable()

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_ANIMATION_RUNNING

    def shutdown(self):
        return ShutDownAnimation()


class ShutDownAnimation(AbstractState):
    def entry(self):
        rospy.loginfo("Motion will shut off")
        speak("Motion will shut off", VALUES.speak_publisher, priority=Speak.HIGH_PRIORITY)
        self.start_animation(rospy.get_param("hcm/animations/shut-down"))

    def evaluate(self):
        if self.animation_finished():
            return ShutDown()

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_SHUT_DOWN

    def shutdown(self):
        # we are shutting down, don't change
        return None


class ShutDown(AbstractState):
    def entry(self):
        speak("Motion says good bye", VALUES.speak_publisher, priority=Speak.HIGH_PRIORITY)
        # give humans the chance to hold the robot before turning it off
        rospy.sleep(3)
        switch_motor_power(False)
        rospy.loginfo("Motion is now off, good bye")

    def evaluate(self):
        # do nothing, the state machine will see that we're finished
        pass

    def exit(self):
        pass

    def hcm_state(self):
        return STATE_SHUT_DOWN

    def shutdown(self):
        # this is shut down
        return None


def switch_motor_power(state):
    """ Calling service from CM730 to turn motor power on or off. But only if not using simulator"""
    #TODO remove "or True" when out of testing
    if rospy.get_param("simulation_active", False):
        rospy.loginfo_throttle(rospy.Duration(2.0), "I'm simulating, not switching motorpower to " + state.__str__())
    else:
        try:
            rospy.wait_for_service("switch_motor_power", timeout=1)
        except rospy.ROSException:
            rospy.logfatal("Can't switch of motorpower, seems like the CM730 is missing.")
            return
        power_switch = rospy.ServiceProxy("switch_motor_power", SwitchMotorPower)
        try:
            response = power_switch(state)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        # wait for motors
        rospy.sleep(1)
