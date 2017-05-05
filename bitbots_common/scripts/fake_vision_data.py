#!/usr/bin/env python3

import rospy
import time
from humanoid_league_msgs.msg import BallInImage, BallsInImage, GoalPartsInImage, GoalInImage, LineInformationInImage, \
    ObstaclesInImage, BallRelative, GoalRelative, ObstacleRelative, ObstaclesRelative, LineInformationRelative, \
    Position2D, Model


class FakeDataGenerator():
    def __init__(self):
        rospy.init_node("fake_data_generator")

        # image space
        self.ball_image_pub = rospy.Publisher("/ball_in_image", BallInImage, queue_size=1)
        self.ball_can_image_pub = rospy.Publisher("/ball_candidates", BallsInImage, queue_size=1)
        self.goal_part_image_pub = rospy.Publisher("/goal_part_candidates", GoalPartsInImage, queue_size=1)
        self.goal_image_pub = rospy.Publisher("/goal_in_image", GoalInImage, queue_size=1)
        self.line_image_pub = rospy.Publisher("/lines_in_image", LineInformationInImage, queue_size=1)
        self.obstacle_image_pub = rospy.Publisher("/obstacles_in_image", ObstaclesInImage, queue_size=1)

        # relative
        self.ball_rel_pub = rospy.Publisher("/ball_relative", BallRelative, queue_size=1)
        self.goal_rel_pub = rospy.Publisher("/goal_relative", GoalRelative, queue_size=1)
        self.obstacle_rel_pub = rospy.Publisher("/obstacles_relative", ObstaclesRelative, queue_size=1)
        self.line_rel_pub = rospy.Publisher("/lines_relative", LineInformationRelative, queue_size=1)

        # position
        self.position_pub = rospy.Publisher("/local_position", Position2D, queue_size=1)

        # model
        self.personal_model_pub = rospy.Publisher("/local_model", Model, queue_size=1)
        self.team_model_pub = rospy.Publisher("/local_model", Model, queue_size=1)

        # data
        self.ball_img_data = [
            # x, y, d, c
            (250, 100, 30, 1), (300, 150, 40, 1)

        ]

        self.ball_can_data = [
            (
                [250, 150],
                [100, 250],
                [30, 50],
                [1, 0.5]
            ),
            (
                [300, 200],
                [150, 180],
                [40, 50],
                [1, 0.5]
            )
        ]

        self.ball_rel_data = [
            # x, y, c
            (1, 0, 1), (1.1, 0, 0.9), (1.2, 0.1, 0.8), (1.3, 0.1, 0.7), (1.4, 0.2, 0.6), (1.5, 0.2, 0.5),
            (1.6, 0.3, 0.4), (1.7, 0.3, 0.4), (1.8, 0.4, 0.4), (1.9, 0.4, 0.4), (2, 0.4, 0.4), (2.1, 0.4, 0.4),
            (2.2, 0.4, 0.4), (2.3, 0.4, 0.4), (2.3, 0.4, 0.4), (2.3, 0.4, 0.4), (2.2, 0.4, 0.5), (2.0, 0.4, 0.6),
            (1.8, 0.4, 0.4), (1.6, 0.4, 0.4), (1.4, 0.4, 0.4), (1.2, 0.4, 0.4), (1.0, 0.4, 0.4), (0.8, 0.4, 0.4)
        ]

        self.goal_rel_data = [
            # lx, ly, rx, ry, cx, cy, c
            (4, 2, 4, 4, 4, 3, 1), (4, 2, 4, 4, 4, 3, 1), (4, 2, 4, 4, 4, 3, 1), (4, 2, 4, 4, 4, 3, 1),
            (4, 2, 4, 4, 4, 3, 1), (4, 2, 4, 4, 4, 3, 1), (4, 2, 4, 4, 4, 3, 1), (4, 2, 4, 4, 4, 3, 1),
            (4, 2, 4, 4, 4, 3, 1), (4, 2, 4, 4, 4, 3, 1), (4, 2, 4, 4, 4, 3, 1), (4, 2, 4, 4, 4, 3, 1),
            (4, 2, 4, 4, 4, 3, 1), (4, 2, 4, 4, 4, 3, 1), (4, 2, 4, 4, 4, 3, 1), (4, 2, 4, 4, 4, 3, 1),
            (3.9, 2, 3.9, 3.9, 4, 3, 1), (3.7, 2, 3.7, 4, 3.7, 3, 1), (3.5, 2, 3.5, 4, 3.5, 3, 1),
            (3.3, 2, 3.3, 4, 3.3, 3, 1), (3.1, 2, 3.1, 4, 3.1, 3, 1), (2.9, 2, 2.9, 4, 2.9, 3, 1),
            (2.7, 2, 2.7, 4, 2.7, 3, 1), (2.5, 2, 2.5, 4, 2.5, 3, 1)
        ]

        self.obstacle_rel_data = [
            # x, y, w, h, t, c
            ([0.5], [-2], [0.5], [0.5], [3], [0.7]), ([0.8], [-2], [0.5], [0.5], [3], [0.7]),
            ([1], [-2], [0.5], [0.5], [3], [0.7]), ([1.2], [-2], [0.5], [0.5], [3], [0.7]),
            ([1.2], [-2.2], [0.5], [0.5], [3], [0.7]), ([1.2], [-2.5], [0.5], [0.5], [3], [0.7]),
            ([1.2], [-2.7], [0.5], [0.5], [3], [0.7]), ([1.0], [-2.5], [0.5], [0.5], [3], [0.7]),
            ([0.8], [-2.3], [0.5], [0.5], [3], [0.7])
        ]

        self.position_data = [
            (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1),
            (1, 1), (1, 1), (1, 1), (1, 1),
            (1.1, 1), (1.3, 1), (1.5, 1), (1.7, 1), (1.9, 1), (2.1, 1), (2.3, 1), (2.5, 1)
        ]

        self.run()

    def run(self):
        i = 0
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            self.ball_image_pub.publish(self.ball_img_msg(*self.ball_img_data[i % len(self.ball_img_data)]))
            self.ball_can_image_pub.publish(
                self.ball_candidates_msg(*self.ball_can_data[i % len(self.ball_can_data)]))
            self.ball_rel_pub.publish(self.ball_rel_msg(*self.ball_rel_data[i % len(self.ball_rel_data)]))
            self.goal_rel_pub.publish(self.goal_rel_msg(*self.goal_rel_data[i % len(self.goal_rel_data)]))
            self.obstacle_rel_pub.publish(
                self.obstacles_rel_msg(*self.obstacle_rel_data[i % len(self.obstacle_rel_data)]))
            self.position_pub.publish(self.position_msg(*self.position_data[i % len(self.position_data)]))
            i += 1
            rate.sleep()

    def ball_candidates_msg(self, xs, ys, ds, cs):
        msg = BallsInImage()
        msg.header.stamp = rospy.Time.from_seconds(time.time())
        i = 0
        cans = []
        for x in xs:
            cans.append(self.ball_img_msg(xs[i], ys[i], ds[i], cs[i]))
            i += 1
        msg.candidates = cans
        return msg

    def ball_img_msg(self, x, y, d, c):
        msg = BallInImage()
        msg.center.x = x
        msg.center.y = y
        msg.diameter = d
        msg.confidence = c
        return msg

    def ball_rel_msg(self, x: int, y: int, c: int):
        msg = BallRelative()
        msg.header.stamp = rospy.Time.from_seconds(time.time())
        msg.ball_relative.x = x
        msg.ball_relative.y = y
        msg.confidence = c
        return msg

    def goal_rel_msg(self, lx, ly, rx, ry, cx, cy, c):
        msg = GoalRelative()
        msg.header.stamp = rospy.Time.from_seconds(time.time())
        msg.left_post.x = lx
        msg.left_post.y = ly
        msg.right_post.x = rx
        msg.right_post.y = ry
        msg.center_direction.x = cx
        msg.center_direction.y = cy
        msg.confidence = c
        return msg

    def obstacle_rel_msg(self, x, y, w, h, t, c):
        msg = ObstacleRelative()
        msg.position.x = x
        msg.position.y = y
        msg.width = w
        msg.height = h
        msg.color = t
        msg.confidence = c
        return msg

    def obstacles_rel_msg(self, xs, ys, ws, hs, ts, cs):
        msg = ObstaclesRelative()
        msg.header.stamp = rospy.Time.from_seconds(time.time())
        i = 0
        obs = []
        for x in xs:
            obs.append(self.obstacle_rel_msg(xs[i], ys[i], ws[i], hs[i], ts[i], cs[i]))
            i += 1
        msg.obstacles = obs
        return msg

    def position_msg(self, x, y):
        msg = Position2D()
        msg.pose.x = x
        msg.pose.y = y
        return msg


if __name__ == "__main__":
    gernerator = FakeDataGenerator()
