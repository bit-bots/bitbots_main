#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import yaml
import rospy
import actionlib
import copy

from pathlib import Path
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from humanoid_league_msgs.msg import Audio, HeadMode
import humanoid_league_msgs.msg
from bitbots_msgs.msg import JointCommand


class JoyNode(object):
    """ This node controls the roboter via Gamepad.
    """

    #TODO read max values from config

    def __init__(self):
        log_level = rospy.DEBUG if rospy.get_param("/debug_active", False) else rospy.INFO
        rospy.init_node("joy_to_twist", log_level=log_level, anonymous=False)

        with open(Path(__file__).parent / "../../config/controller.yaml", "r") as r:
            controller_configs = yaml.safe_load(r)

        self.config = controller_configs[rospy.get_param("~type")] # load the controller specific config

        # --- Initialize Topics ---
        rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=1)
        self.speak_pub = rospy.Publisher('speak', Audio, queue_size=1)
        self.speak_msg = Audio()

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

        self.head_mode_pub = rospy.Publisher("/head_mode", HeadMode, queue_size=1)

        self.head_msg = JointCommand()
        self.head_msg.max_currents = [-1] * 2
        self.head_msg.velocities = [5] * 2
        self.head_msg.accelerations = [40] * 2

        self.head_modes = HeadMode()

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

    def set_head_mode(self, mode):
        msg = HeadMode()
        msg.headMode = mode
        self.head_mode_pub.publish(msg)

    def denormalize_joy(self, gain, axis, msg, deadzone=0):
        if abs(msg.axes[axis]) > deadzone:
            return gain * msg.axes[axis]
        else:
            return 0

    def joy_cb(self, msg):
        # forward and sideward walking with left joystick
        self.walk_msg.linear.x = self.denormalize_joy(
            self.config['walking']['gain_x'],
            self.config['walking']['stick_x'],
            msg, 0.01)
        self.walk_msg.linear.y = self.denormalize_joy(
            self.config['walking']['gain_y'],
            self.config['walking']['stick_y'],
            msg, 0.01)

        # angular walking with shoulder buttons
        turn = msg.axes[self.config['walking']['stick_left']]
        if self.config['walking']['duo_turn']:
            turn -= msg.axes[self.config['walking']['stick_right']]
        turn *= self.config['walking']['gain_turn']

        if turn != 0:
            self.walk_msg.angular.z = turn
        else:
            self.walk_msg.angular.z = 0

        # only publish changes
        if self.walk_msg != self.last_walk_msg:
            self.walk_publisher.publish(self.walk_msg)
        self.last_walk_msg = copy.deepcopy(self.walk_msg)

        # head movement with right joystick
        if rospy.get_param("~head", False):
            pan_goal = self.denormalize_joy(
                self.config['head']['gain_pan'],
                self.config['head']['stick_pan'],
                msg, -1)
            tilt_goal = self.denormalize_joy(
                self.config['head']['gain_tilt'],
                self.config['head']['stick_tilt'],
                msg, -1)

            self.head_msg.joint_names = ["HeadPan", "HeadTilt"]
            self.head_msg.positions = [pan_goal, tilt_goal]
            self.head_pub.publish(self.head_msg)

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
            self.set_head_mode(self.head_modes.BALL_MODE)
        elif msg.buttons[2]:
            # 3
            self.set_head_mode(self.head_modes.FIELD_FEATURES)
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
