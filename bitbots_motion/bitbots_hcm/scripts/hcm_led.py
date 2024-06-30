#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA

from bitbots_msgs.msg import RobotControlState, Strategy

BLINK_DURATION = 0.2
ERROR_TIMEOUT = 1


class HCMLedNode(Node):
    def __init__(self):
        super().__init__("hcm_led")
        self.pub = self.create_publisher(ColorRGBA, "/led1", 1)

        self.color_mapping = {
            RobotControlState.HARDWARE_PROBLEM: (1, 0, 0),
            # RobotControlState.ANIMATION_RUNNING: (0, 0, 0.3),
            RobotControlState.PENALTY: (0.5, 0.5, 0),
            Strategy.ROLE_STRIKER: (0, 1, 0),
            Strategy.ROLE_SUPPORTER: (0.3, 0.3, 0.3),
            Strategy.ROLE_DEFENDER: (0, 0, 1),
            Strategy.ROLE_GOALIE: (0, 0, 0),
            Strategy.ACTION_POSITIONING: (0, 0, 1),
            Strategy.ACTION_GOING_TO_BALL: (0, 1, 0),
            Strategy.ACTION_WAITING: (0, 0, 1),
            Strategy.ACTION_SEARCHING: (0.3, 0.3, 0.3),
        }

        self.last_state = -1

        self.sub_hcm = self.create_subscription(RobotControlState, "robot_state", self.hcm_state_cb, 1)
        self.sub_strategies = self.create_subscription(Strategy, "strategy", self.strategy_state_cb, 1)

    def strategy_state_cb(self, msg: Strategy):
        pass

    def hcm_state_cb(self, msg: RobotControlState):
        state = msg.state

        # Only do something if state changes, to not spam messages
        if state == self.last_state:
            return
        self.last_state = state

        led = ColorRGBA()
        led.a = 1.0

        if state in self.color_mapping:
            r, g, b = self.color_mapping[state]
            led.r, led.g, led.b = float(r), float(g), float(b)
        else:
            self.get_logger().warn(f"Unknown state: {state}")

        self.pub.publish(led)


if __name__ == "__main__":
    rclpy.init(args=None)
    node = HCMLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
