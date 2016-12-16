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
        self.shut_down = False

        self.standupflag = False
        self.soft_off = True
        self.softstart = False
        self.dieflag = False

        self.last_hardware_update = None
        self.last_request = None
        self.start_up_time = rospy.Time.now()

        self.raw_gyro = (0, 0, 0)
        self.smooth_gyro = (0, 0, 0)
        self.not_so_smooth_gyro = (0, 0, 0)

        self.fall_checker = FallChecker()
        self.animation_client = None

        self.softoff_time = rospy.get_param("/softofftime")

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
        return self.soft_off and rospy.Time.now() - self.last_hardware_update > self.softoff_time

    def animation_finished(self):
        return self.animation_client.get_result()

    def say(self, text):
        #todo
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
        VALUES.animation_client.get_result()
        in the evaluate method by yourself.
        :param anim:
        :param follow_state:
        :return:
        """
        self.animation_started = True
        play_animation(anim)


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
            entry_switch = self.state.entry()
            if entry_switch is not None:
                # we directly do another state switch
                self.set_state(entry_switch)
        except:
            self.set_state(self.error_state)

    def evaluate(self):
        """
        Evaluates the current state
        :return:
        """
        switch_state = None

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
    def entry(self):
        pass

    def evaluate(self):
        # leave this if we got a hardware response, or after some time
        # todo zeit parameteriesern?
        if VALUES.last_hardware_update is not None or time.time() - VALUES.start_up_time > 1:
            # check if we directly go into a special state, if not, got to get up
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

    def exit(self, ):
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
                #todo prohibit sudden movement by getting first one time the current motor values
                switch_motor_power(True)
                self.next_state = Record()
                # don't directly change state, we wait for animation to finish
                self.start_animation(rospy.get_param("/animations/motion/walkready"))
                return
            if rospy.Time.now() - VALUES.last_request < 10:  # todo param
                switch_motor_power(True)
                self.next_state = Controlable()
                self.start_animation(rospy.get_param("/animations/motion/walkready"))
                return
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
        #todo
        #walking active
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
        if VALUES.standupflag:
            ## Falling detection
            if VALUES.is_falling():
                return Falling()
            ### Standing up
            if VALUES.is_fallen():
                return Fallen()

        #test if animation should be played
            #stop walking
            #go to animation
        #ask walking for next pose
        pass

    def exit(self):
        pass

    def motion_state(self):
        return STATE_WALKING


class AnimationRunning(AbstractState):
    def entry(self):
        pass

    def evaluate(self):
        #todo
        #if external animation finished
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
        #give humans the chance to hold the robot before turning it off
        rospy.sleep(3)
        switch_motor_power(False)
        rospy.loginfo("Motion is now off, good bye")

    def evaluate(self):
        pass

    def exit(self):
        pass

    def motion_state(self):
        return STATE_SHUT_DOWN


def switch_motor_power(state):
    """ Calling service from CM730 to turn motor power on or off"""
    #todo set motor ram here if turned on, bc it lost it
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
