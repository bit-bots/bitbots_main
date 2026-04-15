#!/usr/bin/env python3

# This script was based on the teleop_twist_keyboard package
# original code can be found at https://github.com/ros-teleop/teleop_twist_keyboard
# The script provides a simple mechanism to test robot behavior in different game states,
# when no game controller is running

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from game_controller_hsl_interfaces.msg import GameState
from game_controller_hsl.utils import get_parameters_from_other_node


class SimGamestate(Node):
    msg = """Setting the GameState by entering a number:

0: STATE_INITIAL = 0
1: STATE_READY = 1
2: STATE_SET = 2
3: STATE_PLAYING =3
4: STATE_FINISHED = 4

5: COMPETITION_TYPE_SMALL = 0
6: COMPETITION_TYPE_MIDDLE = 1
7: COMPETITION_TYPE_LARGE = 3

Set the game phase by entering:
a: GAME_PHASE_TIMEOUT = 0
b: GAME_PHASE_NORMAL = 1
c: GAME_PHASE_EXTRA_TIME = 2
d: GAME_PHASE_PENALTY_SHOOT_OUT = 3

Set play states by entering:
e: SET_PLAY_NONE = 0
f: SET_PLAY_DIRECT_FREE_KICK = 1
g: SET_PLAY_INDIRECT_FREE_KICK = 2
h: SET_PLAY_PENALTY_KICK = 3
i: SET_PLAY_THROW_IN = 4
j: SET_PLAY_GOAL_KICK = 5
k: SET_PLAY_CORNER_KICK = 6

p:     toggle penalized
t:     toggle kicking team
s:     toggle stopped state
+:     increase own score by 1

















CTRL-C to quit
"""

    def __init__(self):
        super().__init__("sim_gamestate")
        self.logger = self.get_logger()

        # Try fetching team id from parameter blackboard or ask user for input
        try:
            self.team_id = get_parameters_from_other_node(self, "parameter_blackboard", ["team_id"])["team_id"]
        except (KeyError, RuntimeError):
            self.logger.error("No team id found in parameter blackboard")
            self.team_id = int(input("Please enter team id: "))

        self.has_kick_off = True

        self.settings = termios.tcgetattr(sys.stdin)

        self.publisher = self.create_publisher(
            GameState,
            "gamestate",
            QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1),
        )

    def loop(self):
        game_state_msg = GameState()
        game_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Init kicking team to our teamID
        game_state_msg.kicking_team = self.team_id

        try:
            print(self.msg)
            while True:
                key = self.get_key()
                if key == "\x03":
                    break
                elif key in ["0", "1", "2", "3", "4"]:
                    int_key = int(key)
                    game_state_msg.main_state = int_key
                elif key in ["5", "6", "7"]:
                    int_key = int(key)
                    game_state_msg.competition_type = int_key - 5
                elif key == "p":  # penalize / unpenalize
                    game_state_msg.penalized = not game_state_msg.penalized
                elif key in [chr(ord("a") + x) for x in range(4)]:
                    game_state_msg.game_phase = ord(key) - ord("a")
                elif key in [chr(ord("e") + x) for x in range(7)]:
                    game_state_msg.set_play = ord(key) - ord("e")
                elif key == "t":
                    if game_state_msg.kicking_team == self.team_id:
                        game_state_msg.kicking_team = self.team_id + 1
                    else:
                        game_state_msg.kicking_team = self.team_id
                elif key == "s":
                    game_state_msg.stopped = not game_state_msg.stopped
                elif key == "+":
                    game_state_msg.own_score += 1

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
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                self.publisher.publish(game_state_msg)
                print(
                    f"""

Competition Type:   {game_state_msg.competition_type}
Game Phase:         {game_state_msg.game_phase}
Set Play:           {game_state_msg.set_play}
Main State:         {game_state_msg.main_state}

Kicking Team:       {game_state_msg.kicking_team}

Penalized:          {game_state_msg.penalized}
Stopped:            {game_state_msg.stopped}

Goals(Own : Rival): {game_state_msg.own_score} : {game_state_msg.rival_score}

CTRL-C to quit
"""
                )

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
