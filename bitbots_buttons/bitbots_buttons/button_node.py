#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
import time
from humanoid_league_msgs.msg import Audio
from std_srvs.srv import Empty

from humanoid_league_speaker.speaker import speak
from std_msgs.msg import Bool
from bitbots_msgs.srv import ManualPenalize
from bitbots_msgs.msg import Buttons


class ButtonNode(Node):
    """ This node handles pressing of buttons on the robot. It should be used to call services on other nodes,
    as an sort of event driven architecture for the buttons.
    """

    def __init__(self):
        super().__init__('bitbots_buttons')

        # --- Params ---
        self.declare_parameter('speak_active', True)
        self.declare_parameter('short_time', 2.0)
        self.declare_parameter('manual_penalty', True)
        self.declare_parameter('in_game', True)
        self.declare_parameter('debounce_time', 0.1)

        self.speaking_active = self.get_parameter('speak_active').get_parameter_value().bool_value
        self.short_time = self.get_parameter('short_time').get_parameter_value().double_value
        self.manual_penality_mode = self.get_parameter('manual_penalty').get_parameter_value().bool_value
        self.in_game_mode = self.get_parameter('in_game').get_parameter_value().bool_value
        self.debounce_time = self.get_parameter('debounce_time').get_parameter_value().double_value

        # --- Class variables ---
        self.button1 = False
        self.button2 = False
        self.button1_time = 0
        self.button2_time = 0

        # --- Initialize Topics ---
        self.create_subscription(Buttons, "/buttons", self.button_cb, 1)
        self.speak_publisher = self.create_publisher(Audio, '/speak', 10)
        self.shoot_publisher = self.create_publisher(Bool, '/shoot_button', 1)

        if self.manual_penality_mode:
            self.manual_penalize_method = self.create_client(ManualPenalize, "manual_penalize")
            while True:
                if self.manual_penalize_method.wait_for_service(timeout_sec=3):
                    continue
                self.get_logger().info("Waiting for manual penalize service.")

        if not self.in_game_mode:
            self.foot_zero_method = self.create_client(Empty, "set_foot_zero")
            while True:
                if self.manual_penalize_method.wait_for_service(timeout_sec=3):
                    continue
                self.get_logger().info("service 'set_foot_zero' not available. Please start ROS Control.")
        self.get_logger().info("Button node running")

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
            if self.manual_penalize_method.service_is_ready():
                self.manual_penalize_method.call_async(0)  # penalize
            else:
                speak("Unpause failed", self.speak_publisher, speaking_active=self.speaking_active)
                self.get_logger().warn("Penalize service did not process request.")

    def button1_long(self):
        """
        Zeroes foot sensors, if foot zero mode is true.
        """
        self.get_logger().warn('Unpause button (1) pressed long')
        speak("1 long", self.speak_publisher, speaking_active=self.speaking_active)
        if self.foot_zero_method.service_is_ready():
            self.foot_zero_method.call_async(1)  # penalize
        else:
            speak("Foot zeroing failed", self.speak_publisher, speaking_active=self.speaking_active)
            self.get_logger().warn("foot zeroing service did not process request.")

    def button1_short(self):
        """
        Penalizes the robot, if it is not penalized and manual penalty mode is true.
        """
        self.get_logger().warn('Pause button (2) pressed short')
        speak("2 short", self.speak_publisher, speaking_active=self.speaking_active)
        if self.manual_penality_mode:
            # switch penalty state by calling service on motion          
            if self.manual_penalize_method.service_is_ready():
                self.manual_penalize_method.call_async(1)  # penalize
            else:
                speak("Pause failed", self.speak_publisher, speaking_active=self.speaking_active)
                self.get_logger().warn("Penalize service did not process request.")


    def button2_long(self):
        """
        Logs that button 2 has been pressed long.
        """
        self.get_logger().warn('Pause button (2) pressed long')
        speak("2 long", self.speak_publisher, speaking_active=self.speaking_active)

def main(args=None):
    rclpy.init(args=args)
    node = ButtonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()