#!/usr/bin/env python2.7
import rospy
from humanoid_league_msgs.msg import Model, BallRelative, ObstaclesRelative, ObstacleRelative
from apriltags2_ros.msg import AprilTagDetection
from apriltags2_ros.msg import AprilTagDetectionArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
import math


class Evaluator(object):
    def __init__(self):
        rospy.init_node("filter_evaluator")

        self.last_ground_truth_position = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.initvals = None
        self.init_initvals = (0, 0)
        self.count = 0

        sub = rospy.Subscriber('local_world_model',
                         Model,
                         self._callback_model,
                         queue_size=1,
                         tcp_nodelay=True)
        gt_pub = rospy.Publisher('ground_truth_marker', Marker, queue_size=1)

        rospy.spin()


    def _callback_model(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform('base_link', 'ball_tag', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('ouch')
        #print(trans)
        self.count += 1
        self.last_ground_truth_position = trans.transform.translation
        marker_msg = Marker()
        marker_msg.type = Marker.POINTS
        marker_msg.action = Marker.ADD
        marker_msg.header.frame_id = 'base_link'
        marker_msg.header.stamp = rospy.Time.now()
        point = Point()
        point.x = self.last_ground_truth_position.x
        point.y = self.last_ground_truth_position.y
        point.z = 0
        marker_msg.points.append(point)
        gt_pub.publish(marker_msg)

        fcnn_pos = msg.ball.ball_relative
        classic_pos = msg.obstacles.obstacles[0].position
        dist_fcnn =  math.sqrt((fcnn_pos.x - self.last_ground_truth_position.x) ** 2 + (fcnn_pos.y - self.last_ground_truth_position.y)**2)# calc distance to ground truth
        dist_classic = math.sqrt((classic_pos.x - self.last_ground_truth_position.x) ** 2 + (classic_pos.y - self.last_ground_truth_position.y)**2) # calc distance
	if not self.initvals:
            if self.count <= 100:
                self.init_initvals = (self.init_initvals[0] + dist_fcnn, self.init_initvals[1] + dist_classic)
            else:
                self.initvals = (self.init_initvals[0] / 100.0, self.init_initvals[1] / 100.0)
                print(self.initvals)
                self.count = 0
        else:
            dist_fcnn = abs(dist_fcnn-self.initvals[0])
            dist_classic = abs(dist_classic-self.initvals[1])
            print('{}, {}, {}, {}, {}'.format(self.count, dist_fcnn, dist_classic, dist_classic-dist_fcnn, dist_fcnn-dist_classic))



if __name__ == "__main__":
    e = Evaluator()

