#!/usr/bin/env python3
# -*- coding: utf8 -*-
import json

import actionlib
import traceback
import rospy
import time
from std_msgs.msg import Header
from bitbots_animation.msg import PlayAnimationResult, PlayAnimationFeedback
from bitbots_animation.msg import PlayAnimationAction as PlayAction
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from bitbots_animation.animation import Animator, parse
from bitbots_common.pose.pypose import PyPose as Pose
from bitbots_animation.srv import AnimationFrame
from sensor_msgs.msg import Imu, JointState
from bitbots_animation.resource_manager import find_animation
from humanoid_league_msgs.msg import Animation, MotionState
from bitbots_common.util.pose_to_message import pose_goal_to_traj_msg


class AnimationNode:
    def __init__(self):
        """Starts a simple action server and waits for requests."""
        log_level = rospy.DEBUG if rospy.get_param("/debug_active", False) else rospy.INFO
        rospy.init_node("bitbots_animation", log_level=log_level, anonymous=False)
        rospy.logdebug("Starting Animation Server")
        server = PlayAnimationAction(rospy.get_name())
        self.current_pose = Pose()
        rospy.spin()


class PlayAnimationAction(object):
    _feedback = PlayAnimationFeedback
    _result = PlayAnimationResult

    def __init__(self, name):
        self.current_pose = Pose()
        self._action_name = name
        self.motion_state = 0

        self.dynamic_animation = rospy.get_param("/animation/dynamic", False)
        robot_type_name = rospy.get_param("/robot_type_name")
        self.used_motor_cids = rospy.get_param("/cm730/" + robot_type_name + "/motors")
        self.used_motor_names = Pose().get_joint_names_cids(self.used_motor_cids)

        # pre defiened messages for performance
        self.anim_msg = Animation()
        self.traj_msg = JointTrajectory()
        self.traj_msg.joint_names = [x.decode() for x in self.used_motor_names]
        self.traj_point = JointTrajectoryPoint()

        rospy.Subscriber("/joint_states", JointState, self.update_current_pose)
        rospy.Subscriber("/motion_state", MotionState, self.update_motion_state)
        self.motion_publisher = rospy.Publisher("/animation", Animation, queue_size=1)

        self._as = actionlib.SimpleActionServer(self._action_name, PlayAction,
                                                execute_cb=self.execute_cb, auto_start=False)

        self._as.start()

    def execute_cb(self, goal):
        """ This is called, when someone calls the animation action"""
        first = True

        # publish info to the console for the user
        rospy.logfatal("Request to play animation %s", goal.animation)

        if self.motion_state != 0 and not goal.motion:  # 0 means controlable
            # we cant play an animation right now
            # but we send a request, so that we may can soon
            self.send_animation_request()

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
        animfunc = animator.playfunc(0.02)  # todo dynamic reconfigure this value
        rate = rospy.Rate(50)
        iteration = 0
        duration_avg = 0
        start = time.time()

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
            if pose is None:
                # todo reset pid values if they were changed in animation - mabye also do this in motion, when recieving finished animation
                # see walking node reset

                # animation is finished
                # tell it to the motion
                self.send_animation(False, True, goal.motion, None)
                self._as.publish_feedback(PlayAnimationFeedback(percent_done=100))
                # we give a positive result
                self._as.set_succeeded(PlayAnimationResult(True))
                return

            self.send_animation(first, False, goal.motion, pose)
            first = False  # we have sent the first frame, all frames after this can't be the first
            perc_done = int(((time.time() - animator.get_start_time()) / animator.get_duration()) * 100)
            perc_done = min(perc_done, 100)
            self._as.publish_feedback(PlayAnimationFeedback(percent_done=perc_done))

            rate.sleep()
            #rospy.sleep(0.01)

            # Count to get the update frequency
            iteration += 1
            if iteration < 100:
                continue

            if duration_avg > 0:
                duration_avg = 0.5 * duration_avg + 0.5 * (time.time() - start)
            else:
                duration_avg = (time.time() - start)

            rospy.logwarn("Updates/Sec %f", iteration / duration_avg)
            iteration = 0
            start = time.time()

    def update_current_pose(self, msg):
        """Gets the current motor positions and updates the representing pose accordingly."""
        names = [x.encode("utf-8") for x in msg.name]
        self.current_pose.set_positions_rad(names, list(msg.position))
        self.current_pose.set_speeds(names, list(msg.velocity))

    def update_motion_state(self, msg):
        self.motion_state = msg.state

    def send_animation_request(self):
        self.anim_msg.request = True
        self.motion_publisher.publish(self.anim_msg)

    def send_animation(self, first, last, motion, pose):
        self.anim_msg.request = False
        self.anim_msg.first = first
        self.anim_msg.last = last
        self.anim_msg.motion = motion
        if pose is not None:
            self.anim_msg.position = pose_goal_to_traj_msg(pose, self.used_motor_names, self.traj_msg, self.traj_point)
        self.motion_publisher.publish(self.anim_msg)


if __name__ == "__main__":
    rospy.logdebug("starting animation node")
    animation = AnimationNode()
