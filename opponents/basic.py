"""Basic opponent simple behavior that stands in the center of the goal."""

from bitbots_body_behavior.simple_behavior import *

GOAL_CENTER = make_pose(-4.5, 0.0)


def main():
    set_head_mode(HEAD_MODE_SEARCH_FIELD_FEATURES)
    set_goal_pose_from_pose(GOAL_CENTER, 0.0)

    while True:
        set_goal_pose_from_pose(GOAL_CENTER, 0.0)
        sleep(0.5)
