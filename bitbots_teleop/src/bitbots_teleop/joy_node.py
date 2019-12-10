#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
import actionlib
import copy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from humanoid_league_msgs.msg import Speak
import humanoid_league_msgs.msg
from sensor_msgs.msg import JointState
from bitbots_msgs.msg import JointCommand


class JoyNode(object):
    """ This node controls the roboter via Gamepad. 
    """

    #TODO read max values from config

    def __init__(self):
        log_level = rospy.DEBUG if rospy.get_param("/debug_active", False) else rospy.INFO
        rospy.init_node("joy_to_twist", log_level=log_level, anonymous=False)

        self.head_pan_pos = 0
        self.head_tilt_pos = 0

        # --- Initialize Topics ---
        rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=1)
        rospy.Subscriber("joint_states", JointState, self.joint_state_cb, queue_size=1)
        self.speak_pub = rospy.Publisher('speak', Speak, queue_size=1)
        self.speak_msg = Speak()
        self.speak_msg.priority = 1

        self.walk_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.walk_msg = Twist()
        self.last_walk_msg = Twist()

        self.walk_msg.linear.x = 0.0
        self.walk_msg.linear.y = 0.0
        self.walk_msg.linear.z = 0.0
        
        self.walk_msg.angular.x = 0.0
        self.walk_msg.angular.y = 0.0
        self.walk_msg.angular.z = 0.0
        
        self.head_pub = rospy.Publisher("/head_motor_goals", JointCommand, queue_size=1)
        self.head_msg = JointCommand()
        self.head_msg.max_currents = [-1] * 2
        self.head_msg.velocities = [5] * 2
        self.head_msg.accelerations = [40] * 2

        # --- init animation action ---
        self.anim_client = actionlib.SimpleActionClient('animation', humanoid_league_msgs.msg.PlayAnimationAction)
        self.anim_goal = humanoid_league_msgs.msg.PlayAnimationGoal()
        self.anim_goal.hcm = False

        first_try = self.anim_client.wait_for_server(rospy.Duration(1))
        
        if not first_try:
            rospy.logerr(
                "Animation Action Server not running! Teleop can not work without animation action server. "
                "Will now wait until server is accessible!")
        self.anim_client.wait_for_server()
        rospy.logwarn("Animation server running, will go on.")


        rospy.spin()

    def play_animation(self, name):
        self.anim_goal.animation = name
        self.anim_client.send_goal(self.anim_goal)
        self.anim_client.wait_for_result()
    
    def send_text(self, text):
        self.speak_msg.text = text
        self.speak_pub.publish(self.speak_msg)
        # don't send it multiple times
        rospy.sleep(0.1)
    
    def joint_state_cb(self, msg):
        i = 0
        for joint_name in msg.name:
            if joint_name == "HeadTilt":
                tilt_number = i
            elif joint_name == "HeadPan":
                pan_number = i
            i+=1
        self.head_pan_pos = msg.position[pan_number]
        self.head_tilt_pos = msg.position[tilt_number]

    def joy_cb(self, msg):
        # forward and sideward walking with left joystick
        if msg.axes[1] > 0:
            self.walk_msg.linear.x = 0.08 * msg.axes[1]
        elif msg.axes[1] < 0:
            self.walk_msg.linear.x = 0.08 * msg.axes[1]
        else:
            self.walk_msg.linear.x = 0

        if msg.axes[0] == 0:
            # to prevent sending a -0
            self.walk_msg.linear.y = 0
        else:
            self.walk_msg.linear.y = 0.08 * msg.axes[0]

        # angular walking with shoulder buttons
        if msg.buttons[6]:
            self.walk_msg.angular.z = 0.5
        elif msg.buttons[7]:
            self.walk_msg.angular.z = -0.5
        else:
            self.walk_msg.angular.z = 0.0

        if msg.axes[2] > 0:
            self.walk_msg.angular.z = 0.6
        elif msg.axes[2] < 0:
            self.walk_msg.angular.z = -0.6

        # only publish changes
        if self.walk_msg != self.last_walk_msg:
            self.walk_publisher.publish(self.walk_msg)
        self.last_walk_msg = copy.deepcopy(self.walk_msg)

        # head movement with right joystick
        """
        joint_names = []
        positions = []
        if pan_goal:
            joint_names.append("HeadPan")
            positions.append(pan_goal)
        if tilt_goal:
            joint_names.append("HeadTilt")
            positions.append(tilt_goal)

        self.head_msg.joint_names = joint_names
        self.head_msg.positions = positions
        if len(joint_names) > 0:
            self.head_pub.publish(self.head_msg)

        """
        # kicking with upper shoulder buttons
        if msg.buttons[4]:
            # L1
            self.play_animation("kick_left")
        elif msg.buttons[5]:
            # R1
            self.play_animation("kick_right")
        # animations with right buttons
        elif msg.buttons[0]:
            # 1
            self.play_animation("cheering")
        elif msg.buttons[1]:
            # 2
            self.send_text("Button not in use")
        elif msg.buttons[2]:
            # 3
            self.send_text("Button not in use")
        elif msg.buttons[3]:
            # 4
            self.send_text("Button not in use")

        # espeak with left buttons
        if msg.axes[5] > 0:
            # up arrow
            self.send_text("Bit Bots four!")
        elif msg.axes[5] < 0:
            # down arrow
            self.send_text("Yaaaaaaaay!")
        elif msg.axes[4] > 0:
            # left arrow
            self.send_text("Goal!")
        elif msg.axes[4] < 0:
            # right arrow
            self.send_text("Thank you university hamburg for funding.")

if __name__ == "__main__":
    joy = JoyNode()
    rospy.spin()
