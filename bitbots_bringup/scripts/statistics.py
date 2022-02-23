#!/usr/bin/env python3

# this script listens to statistic messages, sorts them and republishes them on new topics for better plotting
import rclpy
from rclpy.node import Node
import re
from rosgraph_msgs.msg import TopicStatistics


class StatisticSorter():
    def __init__(self):
        rclpy.init(args=None)

        self.motor_goals_pub = self.create_publisher(TopicStatistics, "stat_motor_goals", 100)
        self.walking_motor_goals_pub = self.create_publisher(TopicStatistics, "stat_walking_motor_goals", 100)
        self.imu_pub = self.create_publisher(TopicStatistics, "stat_imu", 100)
        self.joint_pub = self.create_publisher(TopicStatistics, "stat_joint_states", 100)
        self.animation_pub = self.create_publisher(TopicStatistics, "stat_animation", 100)

        self.motor_goal_number = 0
        self.motor_goals_mean = 0
        self.motor_goals_max = 0

        self.walking_motor_goal_number = 0
        self.walking_motor_goals_mean = 0
        self.walking_motor_goals_max = 0

        self.imu_number = 0
        self.imus_mean = 0
        self.imus_max = 0

        self.joint_state_number = 0
        self.joint_states_mean = 0
        self.joint_states_max = 0

        self.animation_number = 0
        self.animations_mean = 0
        self.animations_max = 0

        rospy.Subscriber("statistics", TopicStatistics, self.stat_cb, queue_size=1000)

        rclpy.spin(self)

        if self.motor_goal_number > 0:
            print("motor")
            print(self.motor_goals_mean/self.motor_goal_number)
            print(self.motor_goals_max)

        if self.walking_motor_goal_number > 0:
            print("walking")
            print(self.walking_motor_goals_mean/self.walking_motor_goal_number)
            print(self.walking_motor_goals_max)

        if self.imu_number > 0:
            print("imu")
            print(self.imus_mean/self.imu_number)
            print(self.imus_max)

        if self.joint_state_number > 0:
            print("joints")
            print(self.joint_states_mean/self.joint_state_number)
            print(self.joint_states_max)

        if self.animation_number > 0:
            print("animation")
            print(self.animations_mean/self.animation_number)
            print(self.animations_max)

    def stat_cb(self, msg: TopicStatistics):
        topic = msg.topic
        pub = msg.node_pub
        sub = msg.node_sub

        #ToDo needs to be tested with changes in namespace
        pattern = re.compile("/record_(0-9)*")
        if pattern.match(sub):
            # test if this is a connection to rosbag record and exclude it
            return

        if topic == "/motor_goals":
            self.motor_goals_pub.publish(msg)
            self.motor_goal_number += 1
            self.motor_goals_mean += rospy.Time.to_nsec(msg.stamp_age_mean)
            self.motor_goals_max = max(self.motor_goals_max, rospy.Time.to_nsec(msg.stamp_age_max))

        if topic == "/walking_motor_goals":
            self.walking_motor_goals_pub.publish(msg)
            self.walking_motor_goal_number += 1
            self.walking_motor_goals_mean += rospy.Time.to_nsec(msg.stamp_age_mean)
            self.walking_motor_goals_max = max(self.walking_motor_goals_max, rospy.Time.to_nsec(msg.stamp_age_max))

        if topic == "/imu":
            self.imu_pub.publish(msg)
            self.imu_number += 1
            self.imus_mean += rospy.Time.to_nsec(msg.stamp_age_mean)
            self.imus_max = max(self.imus_max, rospy.Time.to_nsec(msg.stamp_age_max))

        if topic == "/joint_state":
            self.joint_pub.publish(msg)
            self.joint_state_number += 1
            self.joint_states_mean += rospy.Time.to_nsec(msg.stamp_age_mean)
            self.joint_states_max = max(self.joint_states_max, rospy.Time.to_nsec(msg.stamp_age_max))

        if topic == "/animation":
            self.animation_pub.publish(msg)
            self.animation_number += 1
            self.animations_mean += rospy.Time.to_nsec(msg.stamp_age_mean)
            self.animations_max = max(self.animations_max, rospy.Time.to_nsec(msg.stamp_age_max))


if __name__ == "__main__":
    listener = StatisticSorter()
rter()
