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
space: toggle allowed_to_move
t:     toggle secondary state team

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

    # init secondary state team to our teamID
    gameState.secondaryStateTeam = 8  # this is the default TeamID in the behavior
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
            elif key in [chr(ord('a')+x) for x in range(10)]:
                gameState.secondaryState = ord(key) - ord('a')
            elif key == 't':
                if gameState.secondaryStateTeam == 8:
                    gameState.secondaryStateTeam = 9
                else:
                    gameState.secondaryStateTeam = 8

            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")
            state_publisher.publish(gameState)
            print("Allowed to move:    " + str(gameState.allowedToMove) + "       \nGamestate:          " + str(
                gameState.gameState) + "       \nSecondary State:          " + str(
                gameState.secondaryState) + "       \nSecondary State Team:          " + str(
                gameState.secondaryStateTeam) + "       \nPenalized:          " + str(gameState.penalized) + "            ")

    except Exception as e:
        print(e)

    finally:
        print("\n")

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
