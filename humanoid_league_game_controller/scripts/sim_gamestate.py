#!/usr/bin/env python3

# This script was based on the teleop_twist_keyboard package
# original code can be found at https://github.com/ros-teleop/teleop_twist_keyboard
# The script provides a simple mechanism to test robot behaviour in different game states,
# when no game controller is running

import rospy
from humanoid_league_msgs.msg import GameState as GameStateMsg

import sys
import select
import termios
import tty

msg = """
Setting the GameState by entering a number:

0: GAMESTATE_INITAL=0
1: GAMESTATE_READY=1
2: GAMESTATE_SET=2
3: GAMESTATE_PLAYING=3
4: GAMESTATE_FINISHED=4

p:     toggle penalized
space: toggle allowed_to_move

CTRL-C to quit

"""


def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    return_key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return return_key


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('sim_gamestate')

    state_publisher = rospy.Publisher('gamestate', GameStateMsg, queue_size=1, latch=True)

    gameState = GameStateMsg()
    gameState.header.stamp = rospy.Time.now()

    try:
        print(msg)
        while True:
            key = get_key()
            if key == ' ':
                gameState.allowedToMove = not gameState.allowedToMove
            elif key == '\x03':
                break
            elif key in ['0', '1', '2', '3', '4']:
                int_key = int(key)
                gameState.gameState = int_key
            elif key == 'p':  # penalize / unpenalize
                gameState.penalized = not gameState.penalized

            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")
            state_publisher.publish(gameState)
            print("Allowed to move:    " + str(gameState.allowedToMove) + "       \nGamestate:          " + str(
                gameState.gameState) + "       \nPenalized:          " + str(gameState.penalized) + "            ")

    except Exception as e:
        print(e)

    finally:
        print("\n")

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
