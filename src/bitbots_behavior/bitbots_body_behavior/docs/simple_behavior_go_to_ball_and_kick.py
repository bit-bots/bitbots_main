from math import hypot
from time import sleep

from bitbots_body_behavior.simple_behavior import *

KICK_DISTANCE = 0.6
KICK_DIRECTION_DEG = 0.0


def main():
    set_head_mode(HEAD_MODE_SEARCH_FIELD_FEATURES)
    stop()
    sleep(5)

    while True:
        current_pose = get_current_pose()
        ball_pose = get_ball_pose()

        if current_pose is None or ball_pose is None:
            sleep(0.2)
            continue

        robot_x, robot_y, _ = current_pose
        ball_x, ball_y = ball_pose

        set_goal_pose(ball_x, ball_y)

        if hypot(ball_x - robot_x, ball_y - robot_y) <= KICK_DISTANCE:
            stop()
            kick_ball(KICK_DIRECTION_DEG)
            sleep(4)

        sleep(0.1)
