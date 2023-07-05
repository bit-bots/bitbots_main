#!/usr/bin/env python3

# This script was based on the teleop_twist_keyboard package
# original code can be found at https://github.com/ros-teleop/teleop_twist_keyboard
# The script provides a simple mechanism to test robot behaviour in different game states,
# when no game controller is running

import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from humanoid_league_msgs.msg import GameState as GameStateMsg
from bitbots_utils.utils import get_parameters_from_other_node


class SimGamestate(Node):
    msg = """Setting the GameState by entering a number:
0: GAMESTATE_INITAL=0
1: GAMESTATE_READY=1
2: GAMESTATE_SET=2
3: GAMESTATE_PLAYING=3
4: GAMESTATE_FINISHED=4

Set the secondary game state by entering:
a: STATE_NORMAL = 0
b: STATE_PENALTYSHOOT = 1
c: STATE_OVERTIME = 2
d: STATE_TIMEOUT = 3
e: STATE_DIRECT_FREEKICK = 4
f: STATE_INDIRECT_FREEKICK = 5
g: STATE_PENALTYKICK = 6
h: STATE_CORNER_KICK = 7
i: STATE_GOAL_KICK = 8
j: STATE_THROW_IN = 9

p:     toggle penalized
t:     toggle secondary state team
m:     toggle secondary state mode
k:     toggle kick off









CTRL-C to quit
"""

    def __init__(self):
        super().__init__("sim_gamestate")
        self.logger = self.get_logger()

        self.team_id = get_parameters_from_other_node(self, "parameter_blackboard", ['team_id'])['team_id']
        self.has_kick_off = True

        self.settings = termios.tcgetattr(sys.stdin)

        self.publisher = self.create_publisher(GameStateMsg, f'gamestate', QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1))

    def loop(self):
        game_state_msg = GameStateMsg()
        game_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Init secondary state team to our teamID
        game_state_msg.secondary_state_team = self.team_id

        try:
            print(self.msg)
            while True:
                key = self.get_key()
                if key == '\x03':
                    break
                elif key in ['0', '1', '2', '3', '4']:
                    int_key = int(key)
                    game_state_msg.game_state = int_key
                elif key == 'p':  # penalize / unpenalize
                    game_state_msg.penalized = not game_state_msg.penalized
                elif key in [chr(ord('a') + x) for x in range(10)]:
                    game_state_msg.secondary_state = ord(key) - ord('a')
                elif key == 'm':
                    game_state_msg.secondary_state_mode = (game_state_msg.secondary_state_mode + 1) % 3
                elif key == 't':
                    if game_state_msg.secondary_state_team == self.team_id:
                        game_state_msg.secondary_state_team = self.team_id + 1
                    else:
                        game_state_msg.secondary_state_team = self.team_id
                elif key == 'k':
                    self.has_kick_off = not self.has_kick_off
                game_state_msg.has_kick_off = self.has_kick_off

                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                self.publisher.publish(game_state_msg)
                print(
f"""Penalized:            {game_state_msg.penalized} 
Secondary State Team: {game_state_msg.secondary_state_team}
Secondary State Mode: {game_state_msg.secondary_state_mode}
Secondary State:      {game_state_msg.secondary_state}
Gamestate:            {game_state_msg.game_state}
Has Kick Off:         {game_state_msg.has_kick_off} 


CTRL-C to quit
""")

        except Exception as e:
            print(e)

        finally:
            print()

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        return_key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return return_key


if __name__ == "__main__":
    rclpy.init(args=None)
    node = SimGamestate()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()
