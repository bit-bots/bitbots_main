# -*- coding:utf-8 -*-
import rospy

from bitbots_motion.src.fall_checker import FallChecker


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