from time import sleep

from bitbots_body_behavior.simple_behavior import *  # ignore[r

KICK_DISTANCE = 0.6
KICK_DIRECTION_DEG = 0.0


def main():
    set_head_mode(HEAD_MODE_SEARCH_FIELD_FEATURES)
    stop()
    sleep(5)

    while True:
        current_pose = get_current_pose()
        ball_pose = get_ball_pose()

        if ball_pose is None:
            sleep(0.2)
            continue

        kick_pose = pose_add(ball_pose, make_pose(-0.5, 0.0))
        set_goal_pose_from_pose(kick_pose)

        if pose_distance(current_pose, ball_pose) <= KICK_DISTANCE:
            kick_ball(KICK_DIRECTION_DEG)
            sleep(4)

        sleep(0.1)
