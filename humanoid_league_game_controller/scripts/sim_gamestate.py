#!/usr/bin/env python3

# This script was based on the teleop_twist_keyboard package
# original code can be found at https://github.com/ros-teleop/teleop_twist_keyboard
# The script provides a simple mechanism to test robot behaviour to different gamestates,
# when no gamecontroller is running

import rospy
from humanoid_league_msgs.msg import GameState as GameStateMsg

import sys, select, termios, tty

msg = """
Setting the GameState by entering a number:

0: GAMESTATE_INITAL=0
1: GAMESTATE_READY=1
2: GAMESTATE_SET=2
3: GAMESTATE_PLAYING=3
4: GAMESTATE_FINISHED=4

CTRL-C to quit

"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('sim_gamestate')

    state_publisher = rospy.Publisher('gamestate', GameStateMsg, queue_size=1)

    gameState = GameStateMsg()
    gameState.header.stamp = rospy.Time.now()

    try:
        print(msg)
        while True:
            key = getKey()
            if key == ' ':
                gameState.allowedToMove = not gameState.allowedToMove
            elif key == '\x03':
                break
            elif key in ['0', '1', '2', '3', '4']:
                int_key = int(key)
                gameState.gameState = int_key
            elif key == 'p':  # penalize/unpenalize
                gameState.penalized = not gameState.penalized

            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")
            sys.stdout.write("\x1b[A")
            state_publisher.publish(gameState)
            print ("Allowed to move:    " + str(gameState.allowedToMove) + "       \nGamestate:          " + str(gameState.gameState) + "       \nPenalized:          " + str(gameState.penalized) + "            ")

    except Exception as e:
        print(e)

    finally:
        print("\n")

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


