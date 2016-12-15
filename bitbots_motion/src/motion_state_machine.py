# -*- coding: utf8 -*-
import rospy

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

# todo check if these states can be replaced or add them to the MotionState.msg
STATE_SOFT_OFF = 11
STATE_GETTING_UP_Second = 12
STATE_BUSY = 13

MOTION_STATES = (STATE_CONTROLABLE, STATE_FALLING, STATE_FALLEN, STATE_GETTING_UP, STATE_ANIMATION_RUNNING,
                 STATE_BUSY, STATE_STARTUP, STATE_PENALTY, STATE_PENALTY_ANIMANTION, STATE_RECORD,
                 STATE_SOFT_OFF, STATE_WALKING, STATE_GETTING_UP_Second)

MOVING_STATES = (STATE_CONTROLABLE,
                 STATE_WALKING,
                 STATE_RECORD,
                 STATE_SOFT_OFF)


def state_to_string(state):
    return {
        STATE_CONTROLABLE: "controlable",
        STATE_FALLING: "falling",
        STATE_FALLEN: "fallen",
        STATE_GETTING_UP: "getting up",
        STATE_ANIMATION_RUNNING: "animation running",
        STATE_BUSY: "busy",
        STATE_STARTUP: "startup",
        STATE_PENALTY: "penalty",
        STATE_PENALTY_ANIMANTION: "penalty animation",
        STATE_RECORD: "recording",
        STATE_SOFT_OFF: "soft off",
        STATE_WALKING: "walking",
        STATE_GETTING_UP_Second: "getting up2",
    }.get(state, "unknown state %d" % state)


class MotionStateMachine(object):
    def __init__(self, standupflag):
        self.state = None
        self.set_state(STATE_STARTUP)
        self.penalized = False  # paused
        self.stand_up_handler = FallChecker()
        self.standupflag = standupflag

    def evaluate(self):
        """
        Returns tuple of a boolean and String
        boolean: is controlable
        String: animation to play
        """
        ###
        ### Soft off
        ###
        if self.state == STATE_SOFT_OFF:
            # nothing important is happening, get some rest for the CPU
            rospy.sleep(0.05)
            return

        ###
        ### Penalizing
        ###
        if self.penalized:
            if self.state == STATE_PENALTY:
                # Already penealized
                # turn of motors and wait
                self.switch_motor_power(False)
                rospy.sleep(0.05)
                # update last_cliente time to prohibit shutdown of motion while beeing penalized
                self.last_client_update = rospy.Time.now()
                return (False, None)

            if self.state != STATE_PENALTY_ANIMANTION:
                # We are not yet sitting down, we should start the animation
                rospy.logwarn("Penalized, sitting down!")
                self.set_state(STATE_PENALTY_ANIMANTION)
                # todo service call
                self.animate(
                    rospy.get_param("/animations/motion/penalized"), STATE_PENALTY)
                rospy.logwarn("Motion will not move, I'm penalized")
                return (False, "/animations/motion/penalized")
        elif self.state == STATE_PENALTY:
            # We are not penalized by the gamecontroler anymore, but we are still in STATE_PENALIZED
            # we want to stand up and get into STATE_CONTROLABLE

            # First turn on the motors
            self.switch_motor_power(True)
            # Update the state
            self.set_state(STATE_PENALTY_ANIMANTION)
            # Stand up and get into STATE_CONTROLABLE afterwards
            self.animate("Automatische Walkready", STATE_CONTROLABLE, self.walkready_animation)
            return

        ###
        ### Fall-handling
        ###
        # only do if activated on start
        if self.standupflag:

            ##
            ## Falling detection
            ##
            # check if robot is falling
            falling_pose = self.stand_up_handler.check_falling(self.not_much_smoothed_gyro)
            if falling_pose:
                self.stand_up_handler.set_falling_pose(falling_pose, goal_pose)
                self.set_state(STATE_FALLING)
                return

            ###
            ### Standing up
            ###
            if self.state == STATE_FALLING:
                # maybe the robot is now finished with falling and is lying on the floor
                direction_animation = self.stand_up_handler.check_fallen(self.gyro, self.smooth_gyro,
                                                                         self.robo_angle)
                if direction_animation is not None:
                    self.set_state(STATE_FALLEN)
                    # directly starting to get up. Sending STATE_FALLEN before is still important, e.g. localisation
                    self.set_state(STATE_GETTING_UP)
                    # todo run direction animaiton

    def get_current_state(self):
        # todo those ifs are hacks, remove when possible
        if self.state is STATE_BUSY:
            return STATE_ANIMATION_RUNNING
        elif self.state is STATE_GETTING_UP_Second:
            return STATE_GETTING_UP
        elif self.state is STATE_SOFT_OFF:
            return STATE_CONTROLABLE
        else:
            return self.state

    def set_record(self, rec):
        ###
        ### Recording
        ###
        if rec and not self.state == STATE_RECORD:
            # Recording is requested, but we are not already in this state
            if self.state == STATE_SOFT_OFF:
                # If coming from STATE_SOFT_OFF we want to stand up first
                self.animate(
                    rospy.get_param("/animations/motion/walkready"), STATE_RECORD)
            elif self.state not in (
                    STATE_RECORD,
                    STATE_ANIMATION_RUNNING,
                    STATE_GETTING_UP,
                    STATE_STARTUP,
                    STATE_GETTING_UP_Second):
                # simply set the new state
                self.set_state(STATE_RECORD)
            return
        elif not rec and self.state == STATE_RECORD:
            # Recording finished, go back to normal
            self.set_state(STATE_CONTROLABLE)
            return

    def set_penalized(self, pen):
        self.penalized = pen

    def set_last_client_update(self, time):
        self.last_client_update = time

    def set_state(self, state):
        """ Updatet den internen Status des MotionServers und publiziert
            ihn zum Clienten
        """
        self.state = state
        # unterdrücke state_soft_off nach außen, das sich clients noch trauen die src anzusprechen
        self.publish_motion_state()
        rospy.loginfo("Setting motion state to '%s'" % state_to_string(state))

    def switch_motor_power(self, state):
        """ Calling service from CM730 to turn motor power on or off"""
        rospy.wait_for_service("switch_motor_power")
        power_switch = rospy.ServiceProxy("switch_motor_power", SwitchMotorPower)
        try:
            response = power_switch(state)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
