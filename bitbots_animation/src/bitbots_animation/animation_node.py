#!/usr/bin/env python
# -*- coding: utf8 -*-
import json

import actionlib
import traceback
import rospy
from bitbots_animation.msg import PlayAnimationResult, PlayAnimationFeedback

from bitbots_animation.animation import Animator, parse
from bitbots_animation.srv import AnimationFrame
from sensor_msgs.msg import Imu, JointState
from bitbots_common.util.resource_manager import find_animation  # todo put directly in thins package?


class AnimationNode:
    def __init__(self):
        """Starts a simple action server and waits for requests."""
        log_level = rospy.DEBUG if rospy.get_param("/debug_active", False) else rospy.INFO
        rospy.init_node("bitbots_animation", log_level=log_level, anonymous=False)
        rospy.logdebug("Starting Animation Server")
        server = PlayAnimationAction(rospy.get_name())
        rospy.spin()


def keyframe_service_call(first, last, force, goals):
    """Call the keyframe service of the motion node, to transmit the next keyframe."""
    service_goal = AnimationFrame()
    service_goal.first_frame = first
    service_goal.last_frame = last
    service_goal.force = force
    service_goal.positions = goals
    timeout = rospy.wait_for_service("animation_key_frame", timeout=1)
    if timeout:
        rospy.logwarn("Can't access animation key frame service of motion, can't play my animation.")
        return False
    animation_frame_srv = rospy.ServiceProxy("animation_key_frame", AnimationFrame)
    try:
        response = animation_frame_srv(service_goal)
        return response
    except rospy.ServiceException as exc:
        rospy.logerr("Something went wrong calling the keyframe service.")
        rospy.logerr(traceback.format_exc())
        return False


class PlayAnimationAction(object):
    _feedback = PlayAnimationFeedback
    _result = PlayAnimationResult

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PlayAnimationAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        rospy.loginfo("Will now wait for keyframe service, before providing actions")
        timeout = rospy.wait_for_service("animation_key_frame", timeout=60)
        if timeout:
            rospy.logerr("Didn't found keyframe service, can't run animation. Please start bitbots_motion")
            exit("No motion")
        rospy.loginfo("Found animation_key_frame service, can now start action.")

        rospy.Subscriber("/MotorCurrentPosition", JointState, self.update_current_pose)

        self.dynamic_animation = rospy.get_param("/animation/dynamic")
        self._as.start()

    def execute_cb(self, goal):
        first = True

        # publish info to the console for the user
        rospy.loginfo("Request to play animation %s", goal.animation)

        # start animation
        try:
            if not self.dynamic_animation:
                with open(find_animation(goal.animation)) as fp:
                    animation = parse(json.load(fp))
            else:
                animation = parse(goal.animation)
        except IOError:
            rospy.logwarn("Animation '%s' nicht gefunden" % goal.animation)
            raise
        animator = Animator(animation, self.current_pose)
        animfunc = animator.playfunc(0.025)  # todo dynamic reconfigure this value

        while not rospy.is_shutdown():
            # first check if we have another goal
            if self._as.is_new_goal_available():
                next_goal = self._as.next_goal
                if next_goal.force:
                    rospy.loginfo("Accepted forced animation %s", next_goal.animation)
                    # cancel old stuff and restart
                    self._as.set_preempted(False)
                    self._as.accept_new_goal()
                    return
                else:
                    # can't run this animation now
                    # delete the next goal to make sure, that we can accept something else
                    self._as.next_goal = False
                    self._as.next_goal = None
                    rospy.loginfo("Couldn't start non forced animation, bc another one is already running.")
                    # publish the feedback
                    self._as.set_aborted(False)
                    return

            # if we're here we want to play the next keyframe
            # compute next pose
            pose = animfunc(self.current_pose)
            if pose is None:
                # todo reset pid values if they were changed in animation
                # see walking node reset

                # animation is finished
                # tell it to the motion
                keyframe_service_call(False, True, False, None)
                # we give a positive result
                # todo test if the execute method is done till the end after this
                self._as.set_succeeded(True)

            keyframe_service_call(first, False, goal.force, pose)

    def update_current_pose(self, msg):
        # todo save pose
        self.current_pose = None


if __name__ == "__main__":
    print("starting animation node")
    animation = AnimationNode()