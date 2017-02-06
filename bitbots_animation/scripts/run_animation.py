#!/usr/bin/env python
# -*- coding: utf8 -*-
import actionlib
import rospy

import bitbots_animation.msg


def anim_run():
    anim_client = actionlib.SimpleActionClient('animation', bitbots_animation.msg.PlayAnimationAction)
    rospy.init_node('anim_sender', anonymous=False)
    #rospy.sleep(1)
    anim = rospy.get_param("~anim")
    if anim is None or anim == "":
        rospy.logwarn("Tried to play an animation with an empty name!")
        return False
    first_try = anim_client.wait_for_server(
        rospy.Duration(rospy.get_param("/motion/anim_server_wait_time", 10)))
    if not first_try:
        rospy.logerr(
            "Animation Action Server not running! Motion can not work without animation action server. "
            "Will now wait until server is accessible!")
        anim_client.wait_for_server()
        rospy.logwarn("Animation server now running, motion will go on.")
    goal = bitbots_animation.msg.PlayAnimationGoal()
    goal.animation = anim
    goal.motion = True  # the animation is from the motion
    anim_client.send_goal(goal)

if __name__ == '__main__':
    # run with "rosrun bitbots_animation run_animation.py _text:=TEXT"
    # you can add _prio:=NUMBER with a number between 0 and 2 to set the priority level
    # if you use _filename instead of _text you can give out a file with the corresponding path
    print("Deactivate evaluation fo state machine in motion befor using this")
    anim_run()
