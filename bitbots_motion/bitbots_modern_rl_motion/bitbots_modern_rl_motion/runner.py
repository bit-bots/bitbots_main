#!/usr/bin/env python3

import copy

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.experimental.events_executor import EventsExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy

from bitbots_msgs.msg import Audio, HeadMode, JointCommand


class Runner(Node):
    """This node controls the roboter via Gamepad."""

    # TODO read max values from config

    def __init__(self):
        super().__init__("ml_runner")

        self.declare_parameter("type", "noname")
        self.declare_parameter("head", False)

        # --- Initialize Topics ---
        self.create_subscription(JointCommand, "walk_motor_goals", self.walk_motor_goals_cb, 1)

        self.walk_publisher = self.create_publisher(Twist, "cmd_vel", 1)

        self.walk_msg = Twist()
        self.last_walk_msg = Twist()

        self.walk_msg.linear.x = 0.0
        self.walk_msg.linear.y = 0.0
        self.walk_msg.linear.z = 0.0

        self.walk_msg.angular.x = 0.0
        self.walk_msg.angular.y = 0.0
        self.walk_msg.angular.z = 0.0

        self.head_pub = self.create_publisher(JointCommand, "head_motor_goals", 1)

        self.head_mode_pub = self.create_publisher(HeadMode, "head_mode", 1)

        self.head_msg = JointCommand()
        self.head_msg.max_currents = [-1.0] * 2
        self.head_msg.velocities = [5.0] * 2
        self.head_msg.accelerations = [40.0] * 2

        self.head_modes = HeadMode()

        # --- init animation action ---
        # self.anim_client = ActionClient(self, PlayAnimationAction, 'animation')
        # self.anim_goal = PlayAnimationAction.Goal()
        # self.anim_goal.hcm = False

        # first_try = self.anim_client.wait_for_server(Duration(seconds=1))

        # if not first_try:
        #    self.get_logger().error(
        #        "Animation Action Server not running! Teleop can not work without animation action server. "
        #        "Will now wait until server is accessible!")
        # self.anim_client.wait_for_server()
        # self.get_logger().warn("Animation server running, will go on.")

    def play_animation(self, name):
        return
        self.anim_goal.animation = name
        self.anim_client.send_goal_async(self.anim_goal)
        self.anim_client.wait_for_result()

    def send_text(self, text):
        self.speak_msg.text = text
        self.speak_pub.publish(self.speak_msg)
        # don't send it multiple times
        self.get_clock().sleep_for(Duration(seconds=0.1))

    def set_head_mode(self, mode):
        msg = HeadMode()
        msg.head_mode = mode
        self.head_mode_pub.publish(msg)

    def denormalize_joy(self, gain, axis, msg: Joy, deadzone=0.0):
        if abs(msg.axes[axis]) > deadzone:
            return gain * msg.axes[axis]
        else:
            return 0

    def joy_cb(self, msg: Joy):
        # forward and sideward walking with left joystick
        self.walk_msg.linear.x = float(
            self.denormalize_joy(self.config["walking"]["gain_x"], self.config["walking"]["stick_x"], msg, 0.01)
        )
        self.walk_msg.linear.y = float(
            self.denormalize_joy(self.config["walking"]["gain_y"], self.config["walking"]["stick_y"], msg, 0.01)
        )

        # angular walking with shoulder buttons
        turn = msg.axes[self.config["walking"]["stick_left"]]
        if self.config["walking"]["duo_turn"]:
            turn -= msg.axes[self.config["walking"]["stick_right"]]
        turn *= self.config["walking"]["gain_turn"]

        if turn != 0:
            self.walk_msg.angular.z = float(turn)
        else:
            self.walk_msg.angular.z = 0.0

        # Perform full stop (finish step but do not move afterwards)
        if msg.buttons[self.config["walking"]["btn_full_stop"]] == 1:
            self.walk_msg.linear.x = 0.0
            self.walk_msg.linear.y = 0.0
            self.walk_msg.linear.z = 0.0
            self.walk_msg.angular.z = 0.0
            self.walk_msg.angular.x = -1.0
            # Publish seperatly as we only want to publish the pressing not the releasing
            self.walk_publisher.publish(self.walk_msg)
        else:
            # Undo full stop
            self.walk_msg.angular.x = 0.0

        # only publish changes
        if self.walk_msg.linear != self.last_walk_msg.linear or self.walk_msg.angular.z != self.last_walk_msg.angular.z:
            self.walk_publisher.publish(self.walk_msg)
        self.last_walk_msg = copy.deepcopy(self.walk_msg)

        # head movement with right joystick
        if self.get_parameter("head").get_parameter_value().bool_value:
            pan_goal = float(
                self.denormalize_joy(self.config["head"]["gain_pan"], self.config["head"]["stick_pan"], msg, -1)
            )
            tilt_goal = float(
                self.denormalize_joy(self.config["head"]["gain_tilt"], self.config["head"]["stick_tilt"], msg, -1)
            )

            self.head_msg.joint_names = ["HeadPan", "HeadTilt"]
            self.head_msg.positions = [pan_goal, tilt_goal]
            self.head_pub.publish(self.head_msg)

        if msg.buttons[self.config["kick"]["btn_left"]]:
            self.play_animation("kick_left")
        elif msg.buttons[self.config["kick"]["btn_right"]]:
            self.play_animation("kick_right")
        elif msg.buttons[self.config["misc"]["btn_cheering"]]:
            self.play_animation("cheering")
        elif msg.buttons[self.config["misc"]["btn_say"]]:
            self.send_text("Goal!")


def main():
    rclpy.init(args=None)
    node = Runner()

    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
