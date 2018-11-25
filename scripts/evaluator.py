#!/usr/bin/env python2.7
import rospy
from humanoid_league_messages.msg import Model, BallRelative, ObstaclesRelative, ObstacleRelative
from apriltags2_ros.msg import AprilTagDetection
from apriltags2_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2


class Evaluator(object):
    def __init__(self):
        rospy.init_node("filter_evaluator")

        self.last_ground_truth_position = None

        rospy.Subscriber('local_model',
                         Model,
                         self._callback_model,
                         queue_size=1,
                         tcp_nodelay=True)
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, _callback_detection, queue_size=1, tcp_nodelay=True)

        rospy.spin()


    def _callback_model(self, msg):
        fcnn_pos = msg.ball.ball_relative
        classic_pos = msg.obstacles.obstacles[0].position
        dist_fcnn =  sqrt((fcnn_pos.x - last_ground_truth_position.x) ** 2 + (fcnn_pos.y - last_ground_truth_position.y)**2)# calc distance to ground truth
        dist_classic = sqrt((classic_pos.x - last_ground_truth_position.x) ** 2 + (classic_pos.y - last_ground_truth_position.y)**2) # calc distance
        print('{}, {}'.format(dist_fcnn, dist_classic)

    def _callback_detection(self, msg)
        if msg.detections:
            self.last_ground_truth_position = msg.detections[0].pose.pose.position


if __name__ == "__main__":
    e = Evaluator()

