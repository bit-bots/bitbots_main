#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA

from bitbots_msgs.msg import RobotControlState


BLINK_DURATION = 0.2
ERROR_TIMEOUT = 1

class HCMLedNode(Node):
    def __init__(self):
        rclpy.init(args=None)

        self.pub = self.create_publisher(ColorRGBA, "/led1", 1)

        self.color_mapping = {
            RobotControlState.CONTROLLABLE: (0.3, 0.3, 0.3),
            RobotControlState.FALLING: (0.3, 0.1, 0),
            RobotControlState.FALLEN: (0.3, 0.3, 0),
            RobotControlState.HARDWARE_PROBLEM: (1, 0, 0),
            RobotControlState.ANIMATION_RUNNING: (0, 0, 0.3),
            RobotControlState.WALKING: (0, 0.3, 0),
            RobotControlState.GETTING_UP: (0, 0.3, 0.3),
            RobotControlState.HCM_OFF: (0, 0, 0),
            RobotControlState.MOTOR_OFF: (0.03, 0.03, 0.03),
            RobotControlState.KICKING: (0, 0, 0.1),
            RobotControlState.PENALTY: (0.3, 0, 0.3),
            RobotControlState.PENALTY_ANIMATION: (0.3, 0, 0.3),
            RobotControlState.PICKED_UP: (0, 0.03, 0),
            RobotControlState.RECORD: (0, 0.1, 0),
        }

        self.last_state = -1

        self.sub = self.create_subscription(RobotControlState, "robot_state", self.hcm_state_cb, 1)
        rclpy.spin(self)


    def hcm_state_cb(self, msg: RobotControlState):
        state = msg.state

        # Only do something if state changes, to not spam messages
        if state == self.last_state:
            return
        self.last_state = state

        led = ColorRGBA()
        led.a = 1.0

        if state in self.color_mapping:
            led.r, led.g, led.b = self.color_mapping[state]
        else:
            self.get_logger().warn(f"Unknown state: {state}")

        self.pub.publish(led)
