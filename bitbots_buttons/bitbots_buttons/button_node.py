#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rclpy
from rclpy.node import Node
import time
from bitbots_buttons.msg import Buttons
from humanoid_league_msgs.msg import Audio
from std_srvs.srv import Empty

from humanoid_league_speaker.speaker import speak
from std_msgs.msg import Bool
from bitbots_msgs.srv import ManualPenalize


class ButtonNode(object):
    """ This node handles pressing of buttons on the robot. It should be used to call services on other nodes,
    as an sort of event driven architecture for the buttons.
    """

    def __init__(self):
        log_level = rospy.DEBUG if self.get_parameter('debug_active').get_parameter_value().double_value else rospy.INFO
        rclpy.init(args=None)

        # --- Params ---
        self.speaking_active = self.get_parameter('speak_active').get_parameter_value().double_value
        self.short_time = self.get_parameter('short_time').get_parameter_value().double_value
        self.manual_penality_mode = self.get_parameter('manual_penalty').get_parameter_value().double_value
        self.in_game_mode = self.get_parameter('in_game').get_parameter_value().double_value
        self.debounce_time = self.get_parameter('debounce_time').get_parameter_value().double_value

        # --- Class variables ---
        self.button1 = False
        self.button2 = False
        self.button1_time = 0
        self.button2_time = 0

        # --- Initialize Topics ---
        rospy.Subscriber("/buttons", Buttons, self.button_cb)
        self.speak_publisher = self.create_publisher(Audio, '/speak', 10)
        self.shoot_publisher = self.create_publisher(Bool, '/shoot_button', 1)

        if self.manual_penality_mode:
            self.get_logger().info("Waiting for manual penalize service.")
            try:
                rospy.wait_for_service("manual_penalize", 0.5)
            except rospy.exceptions.ROSException as exc:
                # we can not be sure that the HCM is running, since we may only started the low level software
                pass
            self.manual_penalize_method = self.create_client(ManualPenalize, "manual_penalize")

        if not self.in_game_mode:
            self.get_logger().info("Waiting for foot zeroing service")
            try:
                rospy.wait_for_service("set_foot_zero", 5.0)
            except:
                self.get_logger().error("service 'set_foot_zero' not available. Please start ROS Control.")
            self.foot_zero_method = self.create_client(Empty, "set_foot_zero")

        self.get_logger().info("Button node running")
        rclpy.spin(self)

    def button_cb(self, msg):
        """Callback for msg about pressed buttons."""
        if msg.button1 and not self.button1:
            # button1 was newly pressed
            self.button1 = True
            self.button1_time = float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9)
        elif msg.button2 and not self.button2:
            # button2 was newly pressed
            self.button2 = True
            self.button2_time = float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9)
        elif not msg.button1 and self.button1:
            # button1 not pressed anymore
            self.button1 = False
            current_time = float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9) 
            if current_time - self.button1_time > self.debounce_time:
                if current_time - self.button1_time < self.short_time or self.in_game_mode:
                    self.button1_short()
                else:
                    self.button1_long()
            self.button1_time = 0
        elif not msg.button2 and self.button2:
            # button2 not pressed anymore
            self.button2 = False
            current_time = float(self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]/1e9) 
            if current_time - self.button2_time > self.debounce_time:
                if current_time - self.button2_time < self.short_time or self.in_game_mode:
                    self.button2_short()
                else:
                    self.button2_long()
            self.button2_time = 0

    def button2_short(self):
        """
        Unpenalizes the robot, if it is penalized and manual penalty mode is true.
        """
        self.get_logger().warn('Unpause button (1) pressed short')
        speak("1 short", self.speak_publisher, speaking_active=self.speaking_active)
        self.shoot_publisher.publish(Bool(True))

        if self.manual_penality_mode:
        # switch penalty state by calling service on motion

            try:
                response = self.manual_penalize_method(0)  # unpenalize
            except rospy.ServiceException as exc:
                speak("Unpause failed", self.speak_publisher, speaking_active=self.speaking_active)
                print("Penalize service did not process request: " + str(exc))

    def button1_long(self):
        """
        Zeroes foot sensors, if foot zero mode is true.
        """
        self.get_logger().warn('Unpause button (1) pressed long')
        speak("1 long", self.speak_publisher, speaking_active=self.speaking_active)
        try:
            response = self.foot_zero_method()
        except rospy.ServiceException as exc:
            speak("Foot zeroing failed", self.speak_publisher, speaking_active=self.speaking_active)
            print("foot zeroing service did not process request: " + str(exc))

    def button1_short(self):
        """
        Penalizes the robot, if it is not penalized and manual penalty mode is true.
        """
        self.get_logger().warn('Pause button (2) pressed short')
        speak("2 short", self.speak_publisher, speaking_active=self.speaking_active)
        if self.manual_penality_mode:
            # switch penalty state by calling service on motion

            try:
                response = self.manual_penalize_method(1)  # penalize
            except rospy.ServiceException as exc:
                speak("Pause failed", self.speak_publisher, speaking_active=self.speaking_active)
                print("Penalize service did not process request: " + str(exc))

    def button2_long(self):
        """
        Logs that button 2 has been pressed long.
        """
        self.get_logger().warn('Pause button (2) pressed long')
        speak("2 long", self.speak_publisher, speaking_active=self.speaking_active)

def main():
    ButtonNode()

if __name__ == "__main__":
    main()