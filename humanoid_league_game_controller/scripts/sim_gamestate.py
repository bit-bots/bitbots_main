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
from rclpy.qos import QoSProfile
from humanoid_league_msgs.msg import GameState as GameStateMsg
from bitbots_utils.utils import get_parameters_from_other_node


class SimGamestate(Node):
    msg = """
Setting the GameState by entering a number:

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

CTRL-C to quit


"""

    def __init__(self):
        super().__init__("SimGamestate")
        self.logger = node.get_logger()

        params = get_parameters_from_other_node(self, "parameter_blackboard", ['team_id', 'bot_id'])
        self.team_id = params['team_id']

        self.settings = termios.tcgetattr(sys.stdin)

        namespaces = ['amy', 'rory', 'jack', 'donna', 'rose']
        publishers = [
            self.node.create_publisher(GameStateMsg, f'{n}/gamestate', QoSProfile(durability=1, depth=1))
            for n in namespaces
        ]

        gameState = GameStateMsg()
        gameState.header.stamp = self.node.get_clock().now().to_msg()

        # Init secondary state team to our teamID
        gameState.secondaryStateTeam = self.team_id
        ourTeamID = gameState.secondaryStateTeam

        try:
            print(self.msg)
            while True:
                key = self.get_key()
                if key == '\x03':
                    break
                elif key in ['0', '1', '2', '3', '4']:
                    int_key = int(key)
                    gameState.gameState = int_key
                elif key == 'p':  # penalize / unpenalize
                    gameState.penalized = not gameState.penalized
                elif key in [chr(ord('a') + x) for x in range(10)]:
                    gameState.secondaryState = ord(key) - ord('a')
                elif key == 'm':
                    gameState.secondaryStateMode = (gameState.secondaryStateMode + 1) % 3
                elif key == 't':
                    if gameState.secondaryStateTeam == self.team_id:
                        gameState.secondaryStateTeam = self.team_id + 1
                    else:
                        gameState.secondaryStateTeam = self.team_id

                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                sys.stdout.write("\x1b[A")
                for publisher in publishers:
                    publisher.publish(gameState)

                print(f"""Gamestate:            {gameState.gameState}
Secondary State:      {gameState.secondaryState}
Secondary State Team: {gameState.secondaryStateTeam}
Penalized:            {gameState.penalized}
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

    try:
        # Necessary so that sleep in loop() is not blocking
        thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        thread.start()
        node.loop()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
