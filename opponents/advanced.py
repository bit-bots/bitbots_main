"""Advanced opponent simple behavior that positions itself between the ball and the center of the goal."""

from bitbots_body_behavior.simple_behavior import *

GOAL_CENTER = make_pose(-4.5, 0.0)


def main():
    set_head_mode(HEAD_MODE_SEARCH_FIELD_FEATURES)
    set_goal_pose_from_pose(GOAL_CENTER, 0.0)

    while True:
        ball_pose = get_ball_pose()

        if ball_pose is None:
            set_goal_pose_from_pose(GOAL_CENTER, 0.0)
            sleep(0.1)
            continue

        direction = pose_direction(GOAL_CENTER, ball_pose)
        distance = pose_distance(GOAL_CENTER, ball_pose)
        target_pose = pose_from_direction(GOAL_CENTER, distance * 0.5, direction)

        set_goal_pose_from_pose(target_pose, direction)
        sleep(0.1)
