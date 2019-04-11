#!/usr/bin/env python2.7
"""
Hello World
"""
import rospy


from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
from humanoid_league_msgs.msg import BallRelative
import tf


class TFWorld(object):
    def __init__(self):
        rospy.init_node("world_transformer")
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._callback, queue_size=1, tcp_nodelay=True)
        
        self.robot_pub = rospy.Publisher("amcl_pose", PoseWithCovarianceStamped, queue_size=1, tcp_nodelay=True)
        self.robo_msg = PoseWithCovarianceStamped()

        self.ball_pub = rospy.Publisher("ball_relative", BallRelative, queue_size=1, tcp_nodelay=True)
        self.ball_msg = BallRelative()
        rospy.spin()


    def _callback(self, msg):
        br = tf.TransformBroadcaster()
        for i in range(len(msg.name)):
            if msg.name[i] == "/":
                transform = TransformStamped()
                transform.header.frame_id = "world"
                transform.header.stamp = rospy.Time.now()
                transform.child_frame_id = "base_link"
                transform.transform.translation = msg.pose[i].position
                transform.transform.rotation = msg.pose[i].orientation
                br.sendTransformMessage(transform)

                self.robo_msg.pose.pose = msg.pose[i]
                self.robo_msg.header.stamp = rospy.Time.now()
                self.robo_msg.header.frame_id = "world"
                self.robot_pub.publish(self.robo_msg)

            elif msg.name[i] == "teensize_ball":
                transform = TransformStamped()
                transform.header.frame_id = "world"
                transform.header.stamp = rospy.Time.now()
                transform.child_frame_id = "ball"
                transform.transform.translation = msg.pose[i].position
                transform.transform.rotation = msg.pose[i].orientation
                br.sendTransformMessage(transform)

                self.ball_msg.ball_relative.x = msg.pose[i].position.x
                self.ball_msg.ball_relative.y = msg.pose[i].position.y
                self.ball_msg.header.stamp = rospy.Time.now()
                self.ball_msg.header.frame_id = "world"
                self.ball_pub.publish(self.ball_msg)


        transform = TransformStamped()
        transform.header.frame_id = "world"
        transform.header.stamp = rospy.Time.now()
        transform.child_frame_id = "map"

        transform.transform.translation.x = -10.15/2
        transform.transform.translation.y = -7.13/2
        transform.transform.translation.z = 0
        transform.transform.rotation.x = 0
        transform.transform.rotation.y = 0
        transform.transform.rotation.z = 0
        transform.transform.rotation.w = 1
        br.sendTransformMessage(transform)


if __name__ == "__main__":
    TFWorld()
