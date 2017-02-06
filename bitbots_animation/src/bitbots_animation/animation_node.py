#!/usr/bin/env python
# -*- coding: utf8 -*-
import json

import actionlib
import traceback
import rospy
import time
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


def keyframe_service_call(first, last, motion, pose):
    """Call the keyframe service of the motion node, to transmit the next keyframe."""
    joint_state = JointState()
    if pose is not None:
        rospy.logdebug("send frame to motion: " + str(pose.get_goals()))
    else:
        rospy.logdebug("send empty frame to motion")
    j_header = Header()
    j_header.stamp = rospy.Time.now()
    joint_state.header = j_header
    if pose is not None:
        joint_state.position = pose.get_goals()
        joint_state.name = pose.get_joint_names()
        joint_state.velocity = pose.get_speeds()
    # else:
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
        response = animation_frame_srv(header, first, last, motion, joint_state)
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

        rospy.Subscriber("/joint_states", JointState, self.update_current_pose)

        self.dynamic_animation = rospy.get_param("/animation/dynamic", False)
        self._as.start()

    def execute_cb(self, goal):
        """ This is called, when someone calls the animation action"""
        first = True

        # publish info to the console for the user
        rospy.logfatal("Request to play animation %s", goal.animation)

        # start animation
        try:
            if not self.dynamic_animation:
                with open(find_animation(goal.animation)) as fp:
                    parsed_animation = parse(json.load(fp))
            else:
                parsed_animation = parse(goal.animation)
        except IOError:
            rospy.logwarn("Animation '%s' not found" % goal.animation)
            self._as.set_aborted(False, "Animation not found")
            return
        except ValueError:
            rospy.logwarn("Animation '%s' had an ValueError. Propably there is a syntax error in the animation file. "
                          "See traceback" % goal.animation)
            traceback.print_exc()
            self._as.set_aborted(False, "Animation not found")
            return
        animator = Animator(parsed_animation, self.current_pose)
        animfunc = animator.playfunc(0.025)  # todo dynamic reconfigure this value

        while not rospy.is_shutdown():
            # todo aditional time staying up after shutdown to enable motion to sit down, or play sit down directly?
            # first check if we have another goal
            if self._as.is_new_goal_available():
                next_goal = self._as.next_goal
                rospy.logwarn("New goal: " + next_goal.get_goal().animation)
                if next_goal.get_goal().motion:
                    rospy.logdebug("Accepted motion animation %s", next_goal.get_goal().animation)
                    # cancel old stuff and restart
                    self._as.current_goal.set_aborted()
                    self._as.accept_new_goal()
                    #rospy.sleep(0.1)  # todo this is an anti animation spam device, revaluate its value
                    return
                else:
                    # can't run this animation now
                    self._as.next_goal.set_rejected()
                    # delete the next goal to make sure, that we can accept something else
                    self._as.next_goal = None
                    rospy.logdebug("Couldn't start non motion animation, bc another one is already running.")

            # if we're here we want to play the next keyframe, cause there is no other goal
            # compute next pose
            pose = animfunc(self.current_pose)
            #if pose is not None:
                #rospy.logwarn(pose.get_speeds())
            if pose is None:
                # todo reset pid values if they were changed in animation - mabye also do this in motion, when recieving finished animation
                # see walking node reset

                # animation is finished
                # tell it to the motion
                keyframe_service_call(False, True, goal.motion, None)
                self._as.publish_feedback(PlayAnimationFeedback(percent_done=100))
                # we give a positive result
                self._as.set_succeeded(PlayAnimationResult(True))
                return

            keyframe_service_call(first, False, goal.motion, pose)
            first = False  # we have sent the first frame, all frames after this can't be the first
            perc_done = int(((time.time() - animator.get_start_time()) / animator.get_duration()) * 100)
            perc_done = min(perc_done, 100)
            self._as.publish_feedback(PlayAnimationFeedback(percent_done=perc_done))
            #rospy.sleep(0.01)  #todo this is to give the motion some time to set the motors, evaluate if useful
        #rospy.sleep(0.1)  # todo this is an anti animation spam device, revaluate its value

    def update_current_pose(self, msg):
        """Gets the current motor positions and updates the representing pose accordingly."""
        self.current_pose.set_positions_rad(list(msg.name), list(msg.position))
        self.current_pose.set_speeds(list(msg.name), list(msg.velocity))


if __name__ == "__main__":
    rospy.logdebug("starting animation node")
    animation = AnimationNode()
