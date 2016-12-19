# -*- coding:utf-8 -*-
import time

import rospy
from humanoid_league_msgs.msg import MotionState

from bitbots_motion.src.abstract_state_machine import AbstractState, VALUES, switch_motor_power
from bitbots_motion.src.abstract_state_machine import AbstractStateMachine

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


class MotionStateMachine(AbstractStateMachine):
    def __init__(self, die_flag, standup_flag, soft_off_flag, soft_start, start_test, state_publiser):
        super().__init__()
        VALUES.standup_flag = standup_flag
        VALUES.die_flag = die_flag
        VALUES.soft_off_flag = soft_off_flag
        VALUES.soft_start = soft_start
        VALUES.start_test = start_test
        self.error_state = ShutDown()
        self.set_state(StartupState())
        self.state_publisher = state_publiser

    def publish_state(self, state):
        msg = MotionState()
        msg.state = state.motion_state()
        self.state_publisher.publish(msg)


class StartupState(AbstractState):
    def entry(self):
        pass

    def evaluate(self):
        # leave this if we got a hardware response, or after some time
        # todo zeit parameteriesern?
        if VALUES.last_hardware_update is not None or time.time() - VALUES.start_up_time > 1:
            # check if we directly go into a special state, if not, got to get up
            if VALUES.start_test:
                #todo
                pass
                return
            if VALUES.record:
                switch_motor_power(True)
                return Record()
            if VALUES.softstart:
                switch_motor_power(False)
                VALUES.last_client_update = rospy.Time.now() - 120
                return Softoff()
            switch_motor_power(True)
            if VALUES.penalized:
                return PenaltyAnimationIn()

            # normal states
            if VALUES.standupflag:
                return GettingUp()
            else:
                return Controlable()

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
                self.start_animation(rospy.get_param("/animations/motion/walkready"))
                return
            if rospy.Time.now() - VALUES.last_request < 10:  # todo param
                #got a new move request
                switch_motor_power(True)
                self.next_state = Controlable()
                self.start_animation(rospy.get_param("/animations/motion/walkready"))
                return
            if VALUES.is_die_time():
                return ShutDown()
            rospy.sleep(0.1)
        else:
            if VALUES.animation_finished():
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
            return Controlable()

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
        if VALUES.animation_finished():
            return self.next_state

    def exit(self):
        pass

    def motion_state(self):
        return STATE_PENALTY_ANIMANTION


class PenaltyAnimationOut(AbstractState):
    def entry(self):
        rospy.logwarn("Not penalized anymore, getting up!")
        self.next_state = Controlable()
        self.start_animation(rospy.get_param("/animations/motion/penalized_end"))

    def evaluate(self):
        # wait for animation started in entry
        if VALUES.animation_client.get_result():
            return self.next_state

    def exit(self):
        pass

    def motion_state(self):
        return STATE_PENALTY_ANIMANTION


class Penalty(AbstractState):
    def entry(self):
        switch_motor_power(False)
        rospy.loginfo("Im now penalized")

    def evaluate(self):
        if VALUES.penalized:
            # still penalized, lets wait a bit
            rospy.sleep(0.05)
            # prohibit soft off
            VALUES.last_client_update = rospy.Time.now()
        else:
            return PenaltyAnimationOut()

    def exit(self):
        switch_motor_power(True)

    def motion_state(self):
        return STATE_PENALTY


class GettingUp(AbstractState):
    def entry(self):
        rospy.logdebug("Getting up!")
        self.next_state = Controlable()
        self.start_animation(
            VALUES.fall_checker.check_fallen(VALUES.raw_gyro, VALUES.smooth_gyro))

    def evaluate(self):
        # wait for animation started in entry
        if VALUES.animation_finished():
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


class Controlable(AbstractState):
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
                return Controlable()

    def evaluate(self):
        if VALUES.animation_finished():
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
        if VALUES.animation_finished():
            return self.next_state

    def exit(self):
        pass

    def motion_state(self):
        return STATE_FALLEN


class Walking(AbstractState):
    def entry(self):
        # todo
        # walking active
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

                # test if animation should be played
                # stop walking
                # go to animation
        # ask walking for next pose
        pass

    def exit(self):
        pass

    def motion_state(self):
        return STATE_WALKING


class AnimationRunning(AbstractState):
    def entry(self):
        pass

    def evaluate(self):
        # todo
        # if external animation finished
        return Controlable()
        pass

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
        if VALUES.animation_finished():
            return

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
        pass

    def exit(self):
        pass

    def motion_state(self):
        return STATE_SHUT_DOWN
