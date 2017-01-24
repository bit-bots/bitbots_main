#!/usr/bin/env python
# -*- coding: utf8 -*-
import json

import actionlib
import traceback
import rospy
from std_msgs.msg import Header
from bitbots_animation.msg import PlayAnimationResult, PlayAnimationFeedback
from bitbots_animation.msg import PlayAnimationAction as PlayAction
from trajectory_msgs.msg import JointTrajectoryPoint

from bitbots_animation.animation import Animator, parse
from bitbots_common.pose.pypose import PyPose as Pose
from bitbots_animation.srv import AnimationFrame
from sensor_msgs.msg import Imu, JointState
from bitbots_animation.resource_manager import find_animation


class AnimationNode:
    def __init__(self):
        """Starts a simple action server and waits for requests."""
        log_level = rospy.DEBUG if rospy.get_param("/debug_active", False) else rospy.INFO
        rospy.init_node("bitbots_animation", log_level=log_level, anonymous=False)
        rospy.logdebug("Starting Animation Server")
        server = PlayAnimationAction(rospy.get_name())
        self.current_pose = Pose()
        rospy.spin()


def keyframe_service_call(first, last, force, pose):
    """Call the keyframe service of the motion node, to transmit the next keyframe."""
    joint_state = JointState()
    if pose != None:
        rospy.logwarn(pose.get_positions())
    else:
        rospy.logwarn(None)
    j_header = Header()
    j_header.stamp = rospy.Time.now()
    joint_state.header = j_header
    if pose is not None:
        joint_state.position = pose.get_positions()
        joint_state.name = pose.get_joint_names()
    #else:
    #    joint_state.positions = []
    #    joint_state.name = []
    header = Header()
    header.stamp = rospy.Time.now()

    timeout = rospy.wait_for_service("animation_key_frame", timeout=1)
    if timeout:
        rospy.logwarn("Can't access animation key frame service of motion, can't play my animation.")
        return False
    animation_frame_srv = rospy.ServiceProxy("animation_key_frame", AnimationFrame)
    try:
        response = animation_frame_srv(header, first, last, force, joint_state)
        return response
    except rospy.ServiceException as exc:
        rospy.logerr("Something went wrong calling the keyframe service.")
        rospy.logerr(traceback.format_exc())
        return False


class PlayAnimationAction(object):
    _feedback = PlayAnimationFeedback
    _result = PlayAnimationResult

    def __init__(self, name):
        self.current_pose = Pose()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PlayAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        rospy.loginfo("Will now wait for keyframe service, before providing actions")
        timeout = rospy.wait_for_service("animation_key_frame", timeout=60)
        if timeout:
            rospy.logerr("Didn't found keyframe service, can't run animation. Please start bitbots_motion")
            exit("No motion")
        rospy.loginfo("Found animation_key_frame service, can now start action.")

        rospy.Subscriber("/current_motor_positions", JointState, self.update_current_pose)

        self.dynamic_animation = rospy.get_param("/animation/dynamic", False)
        self._as.start()

    def execute_cb(self, goal):
        """ This is called, when someone calls the animation action"""
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
            rospy.logwarn("Animation '%s' not found" % goal.animation)
            self._as.set_aborted(False, "Animation not found")
            return
        animator = Animator(animation, self.current_pose)
        animfunc = animator.playfunc(0.025)  # todo dynamic reconfigure this value

        while not rospy.is_shutdown():
            # first check if we have another goal
            if self._as.is_new_goal_available():
                next_goal = self._as.next_goal
                rospy.logwarn("New goal: " + next_goal.animation)
                if next_goal.force:
                    rospy.logdebug("Accepted forced animation %s", next_goal.animation)
                    # cancel old stuff and restart
                    self._as.accept_new_goal()
                    return
                else:
                    # can't run this animation now
                    self._as.next_goal.set_rejected()
                    # delete the next goal to make sure, that we can accept something else
                    self._as.next_goal = None
                    rospy.logdebug("Couldn't start non forced animation, bc another one is already running.")

            # if we're here we want to play the next keyframe, cause there is no other goal
            # compute next pose
            pose = animfunc(self.current_pose)
            if pose is None:
                # todo reset pid values if they were changed in animation - mabye also do this in motion, when recieving finished animation
                # see walking node reset

                # animation is finished
                # tell it to the motion
                keyframe_service_call(False, True, False, None)
                # we give a positive result
                # todo test if the execute method is done till the end after this; update:I think its okay
                self._as.set_succeeded(PlayAnimationResult(True))
                return

            keyframe_service_call(first, False, goal.force, pose)
            self._as.publish_feedback(PlayAnimationFeedback(percent_done=0)) #todo compute feedback in percent, int

    def update_current_pose(self, msg):
        """Gets the current motor positions and updates the representing pose accordingly."""
        self.current_pose.set_positions(msg.name, msg.position)
        self.current_pose.set_speeds(msg.name, msg.velocity)


if __name__ == "__main__":
    rospy.logdebug("starting animation node")
    animation = AnimationNode()
