#!/usr/bin/env python3

from typing import Optional

import matplotlib.colors as colors
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA

from bitbots_msgs.msg import RobotControlState, Strategy


def color_to_msg(color: str) -> ColorRGBA:
    """
    Converts a color string to a ColorRGBA message
    """
    r, g, b = colors.to_rgb(color)
    return ColorRGBA(r=r, g=g, b=b, a=1.0)


class HCMLedNode(Node):
    def __init__(self):
        super().__init__("hcm_led")

        # Publishers for the core leds
        self.pub_core_0 = self.create_publisher(ColorRGBA, "/led0", 1)
        self.pub_core_1 = self.create_publisher(ColorRGBA, "/led1", 1)
        self.pub_core_2 = self.create_publisher(ColorRGBA, "/led2", 1)

        # Publishers for the imu leds
        self.pub_imu_3 = self.create_publisher(ColorRGBA, "/led3", 1)
        self.pub_imu_4 = self.create_publisher(ColorRGBA, "/led4", 1)
        self.pub_imu_5 = self.create_publisher(ColorRGBA, "/led5", 1)

        self.stratedy_msg: Optional[Strategy] = None
        self.robot_controll_state_msg: Optional[RobotControlState] = None

        # Maps the robot control state to a color
        self.color_mapping_robot_controll_state = {
            RobotControlState.HARDWARE_PROBLEM: color_to_msg("red"),
            RobotControlState.PENALTY: color_to_msg("yellow"),
        }

        # Maps the strategy role to a color. Ideally the colors of the role and action should match
        # for default behavoior. For example, the striker role should have the same color as the going to ball action.
        # As the stiker is the most likely to go to the ball.
        self.color_mapping_strategy_role = {
            Strategy.ROLE_STRIKER: color_to_msg("green"),
            Strategy.ROLE_SUPPORTER: color_to_msg("dimgrey"),
            Strategy.ROLE_DEFENDER: color_to_msg("blue"),
            Strategy.ROLE_GOALIE: color_to_msg("black"),
        }

        # Maps the strategy action to a color
        self.color_mapping_strategy_action = {
            Strategy.ACTION_POSITIONING: color_to_msg("blue"),
            Strategy.ACTION_GOING_TO_BALL: color_to_msg("green"),
            Strategy.ACTION_WAITING: color_to_msg("blue"),
            Strategy.ACTION_SEARCHING: color_to_msg("dimgrey"),
        }

        # Subscribes to the robot control state and strategy
        self.sub_hcm = self.create_subscription(RobotControlState, "robot_state", self.hcm_state_cb, 1)
        self.sub_strategies = self.create_subscription(Strategy, "strategy", self.strategy_state_cb, 1)

    def strategy_state_cb(self, msg: Strategy):
        # If the message is the same as the last one, ignore it
        if self.stratedy_msg == msg:
            return
        self.stratedy_msg = msg
        # Change the led color accordingly
        self.update_leds()

    def hcm_state_cb(self, msg: RobotControlState):
        # If the message is the same as the last one, ignore it
        if self.robot_controll_state_msg == msg:
            return
        self.robot_controll_state_msg = msg
        # Change the led color accordingly
        self.update_leds()

    def update_leds(self):
        """
        Updates the leds based on the current state. The priority is as follows: robot control state > strategy role and action.
        The robot controll state changes all leds to the same color. The strategy role and action changes the led on the left
        of the imu and coreboard and the stategy action changes the mittle and right led on the imu and coreboard.
        """
        if (
            self.robot_controll_state_msg is not None
            and self.robot_controll_state_msg.state in self.color_mapping_robot_controll_state
        ):
            # If the robot control state is in the mapping, change all leds to the color of the state
            color = self.color_mapping_robot_controll_state[self.robot_controll_state_msg.state]
            self.pub_core_0.publish(color)
            self.pub_core_1.publish(color)
            self.pub_core_2.publish(color)
            self.pub_imu_3.publish(color)
            self.pub_imu_4.publish(color)
            self.pub_imu_5.publish(color)
            return

        if self.stratedy_msg is not None and self.stratedy_msg.role in self.color_mapping_strategy_role:
            # If the strategy role is in the mapping, change the left led on the imu and coreboard to the color of the role
            color = self.color_mapping_strategy_role[self.stratedy_msg.role]
            self.pub_core_0.publish(color)
            self.pub_imu_3.publish(color)

        if self.stratedy_msg is not None and self.stratedy_msg.action in self.color_mapping_strategy_action:
            # If the strategy action is in the mapping, change the middle and right led on the imu and coreboard to the color of the action
            color = self.color_mapping_strategy_action[self.stratedy_msg.action]
            self.pub_core_1.publish(color)
            self.pub_core_2.publish(color)
            self.pub_imu_4.publish(color)
            self.pub_imu_5.publish(color)


if __name__ == "__main__":
    rclpy.init(args=None)
    node = HCMLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
