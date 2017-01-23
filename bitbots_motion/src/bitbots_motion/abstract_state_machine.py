# -*- coding:utf-8 -*-
import traceback

import rospy

from bitbots_motion.values import VALUES
import bitbots_animation.msg
from std_msgs.msg import String


class AbstractState(object):
    def __init__(self):
        # bool, is an animation started, where we want to wait for its finish
        self.animation_started = False
        self.next_state = None

    def entry(self):
        msg = "You should overwrite entry() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def exit(self):
        """
        You can't change states in the exit method
        """
        msg = "You should overwrite exit() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def evaluate(self):
        """
        Let's the state run one step
        """
        msg = "You should overwrite evaluate() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def motion_state(self):
        """Gives back the name that will be displayed in the motion state message to the outside"""
        msg = "You should overwrite motion_state() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def shutdown(self):
        """Tells the state machine to which state it shall go, if there is an external shutdown.
        Should normally be ShutDown or ShutDownAnimation."""
        msg = "You should overwrite shutdown() in %s" % self.__class__.__name__
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
        rospy.loginfo("Playing animation " + anim_name)
        if anim_name is None or anim_name == "empty":
            rospy.logwarn("Tried to play an animation with an empty name!")
            return False
        first_try = VALUES.animation_client.wait_for_server(
            rospy.Duration(rospy.get_param("/motion/anim_server_wait_time", 10)))
        if not first_try:
            rospy.logerr(
                "Animation Action Server not running! Motion can not work without animation action server. "
                "Will now wait until server is accessible!")
            VALUES.animation_client.wait_for_server()
            rospy.logwarn("Animation server now running, motion will go on.")
        goal = bitbots_animation.msg.PlayAnimationGoal()
        goal.animation = anim_name
        VALUES.animation_client.send_goal(goal)
        return True

    def animation_finished(self):
        if self.animation_started and VALUES.animation_client.get_state() != 9:
            # We started an animation, let's see if it is finished
            return VALUES.animation_client.get_result()
        else:
            # We didn't actually started an animation, probably due to some error before, print warning and go on
            rospy.logwarn_throttle(1, "Tried to ask if animation is finished, but no animation was started.")
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
        if self.state is not None:
            self.publish_state(state)
            self.state.exit()
        self.state = state
        if self.debug_active:
            # publish name of the current state if debug is active
            self.debug_publisher.publish(self.state.__class__.__name__)
            rospy.logdebug("Current state: " + self.state.__class__.__name__)
        entry_switch = self.state.entry()
        if entry_switch is not None:
            # we directly do another state switch
            self.set_state(entry_switch)

    def evaluate(self):
        """
        Evaluates the current state
        :return:
        """

        # do we need to shut down the motion
        if VALUES.shut_down:
            # get shutdown state of current state
            shutdown_state = self.state.shutdown()
            if shutdown_state is None:
                # We're already in a shut down state, there is nothing left to do for us
                return
            else:
                # set the state
                self.set_state(shutdown_state)
                return

        switch_state = self.state.evaluate()

        if switch_state is not None:
            if switch_state.__class__ in self.connections[self.state.__class__]:
                self.set_state(switch_state)
            else:
                print(
                    "Not allowed transistion from " + self.state.__class__.__name__ + " to " +
                    switch_state.__class__.__name__)

    def publish_state(self, state):
        msg = "You should overwrite publish_state() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def is_shutdown(self):
        msg = "You should overwrite is_shutdown() in %s" % self.__class__.__name__
        raise NotImplementedError(msg)

    def get_current_state(self):
        return self.state.motion_state()
