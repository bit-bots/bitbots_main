#!/usr/bin/env python3
import rospy

from humanoid_league_msgs.msg import BallsInImage, GoalInImage, LineInformationInImage, ObstaclesInImage


class TimeFixer:
    def __init__(self):
        rospy.init_node('jetson_time_fixer')

        rospy.Subscriber('/ball_in_image', BallsInImage, self.ball_cb, queue_size=40)
        rospy.Subscriber('/goal_in_image', GoalInImage, self.goal_cb, queue_size=40)
        rospy.Subscriber('/line_in_image', LineInformationInImage, self.line_cb, queue_size=40)
        rospy.Subscriber('/obstacles_in_image', ObstaclesInImage, self.obstacle_cb, queue_size=40)

        self.ball_pub = rospy.Publisher('/ball_in_image_fixed', BallsInImage, queue_size=40)
        self.goal_pub = rospy.Publisher('/goal_in_image_fixed', GoalInImage, queue_size=40)
        self.line_pub = rospy.Publisher('/line_in_image_fixed', LineInformationInImage, queue_size=40)
        self.obstacle_pub = rospy.Publisher('/obstacles_in_image_fixed', ObstaclesInImage, queue_size=40)

        rospy.spin()

    def ball_cb(self, msg: BallsInImage):
        msg.header.stamp = rospy.Time.now()
        self.ball_pub.publish(msg)

    def goal_cb(self, msg: GoalInImage):
        msg.header.stamp = rospy.Time.now()
        self.goal_pub.publish(msg)

    def line_cb(self, msg: LineInformationInImage):
        msg.header.stamp = rospy.Time.now()
        self.line_pub.publish(msg)

    def obstacle_cb(self, msg: ObstaclesInImage):
        msg.header.stamp = rospy.Time.now()
        self.obstacle_pub.publish(msg)


if __name__ == "__main__":
    TimeFixer()