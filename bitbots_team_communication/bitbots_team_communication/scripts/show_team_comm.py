#!/usr/bin/env python3

import sys
import threading

import rclpy
from rclpy.constants import S_TO_NS
from rclpy.node import Node
from rclpy.time import Duration, Time
from transforms3d.euler import quat2euler

from bitbots_msgs.msg import Strategy, TeamData

print_msg = """
Team Communication Visualizer
-----------------------------
Robot 1
    Outdated: True
    Time since message: 100
    State: UNKNOWN
    Position
        x: 00.0
        y: 00.0
        yaw: 00.0
        x_cov: 00.0
        y_conv: 00.0
        yaw_cov: 00.0
    Ball
        x: 00.00
        y: 00.00
        x_cov: 00.00
        y_cov: 00.00
    Obstacles found: 0
    Strategy
        Role: UNDEFINED
        Action: UNDEFINED
        SIDE: UNDEFINED
"""


class TeamCommPrinter(Node):
    def __init__(self):
        super().__init__("show_team_comm")
        self.subscriber = self.create_subscription(TeamData, "team_data", self.data_cb, 1)

        self.team_data = {}
        for i in range(1, 5):
            self.team_data[i] = TeamData()
            self.team_data[i].robot_id = i

        self.states = {
            TeamData.STATE_UNKNOWN: "Unknown",
            TeamData.STATE_PENALIZED: "Penalized",
            TeamData.STATE_UNPENALIZED: "Unpenalized",
        }
        self.roles = {
            Strategy.ROLE_UNDEFINED: "Undefined",
            Strategy.ROLE_GOALIE: "Goalie",
            Strategy.ROLE_STRIKER: "Striker",
            Strategy.ROLE_OTHER: "Other",
            Strategy.ROLE_IDLING: "Idle",
            Strategy.ROLE_DEFENDER: "Defender",
            Strategy.ROLE_SUPPORTER: "Supporter",
        }
        self.actions = {
            Strategy.ACTION_KICKING: "Kicking",
            Strategy.ACTION_SEARCHING: "Searching",
            Strategy.ACTION_LOCALIZING: "Localizing",
            Strategy.ACTION_GOING_TO_BALL: "Going To Ball",
            Strategy.ACTION_WAITING: "Waiting",
            Strategy.ACTION_POSITIONING: "Positioning",
            Strategy.ACTION_TRYING_TO_SCORE: "Trying to score",
            Strategy.ACTION_UNDEFINED: "Undefined",
        }
        self.sides = {
            Strategy.SIDE_LEFT: "Left",
            Strategy.SIDE_RIGHT: "Right",
            Strategy.SIDE_MIDDLE: "Middle",
            Strategy.SIDE_UNDEFINED: "Undefined",
        }

        self.run_spin_in_thread()

    def data_cb(self, msg: TeamData):
        self.team_data[msg.robot_id] = msg

    def generate_string(self, data: TeamData):
        lines = []
        lines.append(f"Robot {data.robot_id}")
        duration = self.get_clock().now() - Time.from_msg(data.header.stamp)
        time = min(100, round(duration.nanoseconds / S_TO_NS))
        lines.append(f"Time since message: {time:<3}")
        lines.append(f"State: {self.states[data.state]:<11}")
        lines.append("Position")
        lines.append(f"  x: {round(data.robot_position.pose.position.x, 3):<4}")
        lines.append(f"  y: {round(data.robot_position.pose.position.y, 3):<4}")
        quat_xyzw = data.robot_position.pose.orientation
        rpy = quat2euler((quat_xyzw.w, quat_xyzw.x, quat_xyzw.y, quat_xyzw.z))
        lines.append(f"  yaw: {round(rpy[2], 3):>4}")
        lines.append(f"  x_cov: {round(data.robot_position.covariance[0], 3):<4}")
        lines.append(f"  y_cov: {round(data.robot_position.covariance[7], 3):<4}")
        lines.append(f"  yaw_cov: {round(data.robot_position.covariance[35], 3):<4}")
        lines.append("Ball absolute")
        lines.append(f"  x: {round(data.ball_absolute.pose.position.x, 3):<4}")
        lines.append(f"  y: {round(data.ball_absolute.pose.position.y, 3):<4}")
        lines.append(f"  x_cov: {round(data.ball_absolute.covariance[0], 3):<4}")
        lines.append(f"  y_cov: {round(data.ball_absolute.covariance[7], 3):<4}")
        lines.append(f"Robots found: {len(data.robots.robots)}")
        lines.append("Strategy")
        lines.append(f"  Role: {self.roles[data.strategy.role]:<9}")
        lines.append(f"  Action: {self.actions[data.strategy.action]:<15}")
        lines.append(f"  Side: {self.sides[data.strategy.offensive_side]:<9}")
        return lines

    def run(self):
        first = True
        while rclpy.ok():
            prints = []
            # generate text to display for each robot
            for data in self.team_data.values():
                prints.append(self.generate_string(data))
            number_of_line = len(prints[0])
            max_line_length = 30
            # remove old text
            if not first:
                sys.stdout.write("\x1b[A" * number_of_line)
            first = False
            for i in range(number_of_line):
                line = ""
                for j in range(len(prints)):
                    # append line with additional space to keep same length
                    line = line + prints[j][i] + (max_line_length - len(prints[j][i])) * " "
                print(f"{line}")

            self.get_clock().sleep_for(Duration(seconds=1))

    def run_spin_in_thread(self):
        # Necessary in ROS2, else we are forever stuck
        thread = threading.Thread(target=rclpy.spin, args=[self], daemon=True)
        thread.start()


if __name__ == "__main__":
    rclpy.init(args=None)
    printer = TeamCommPrinter()
    printer.run()
