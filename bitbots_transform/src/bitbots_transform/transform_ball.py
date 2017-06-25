#!/usr/bin/env python2.7
import rospy
from bitbots_transform.transform_helper import transf
from humanoid_league_msgs.msg import BallRelative, BallInImage, BallsInImage
from sensor_msgs.msg import CameraInfo


class TransformBall(object):
    def __init__(self):
        rospy.Subscriber("ball_in_image", BallsInImage, self._callback_ball, queue_size=1)
        rospy.Subscriber("camera/camera_info", CameraInfo, self._callback_camera_info)
        self.ball_relative_pub = rospy.Publisher("ball_relative", BallRelative, queue_size=10)
        self.caminfo = None  # type:CameraInfo

        rospy.init_node("transform_ball")

        rospy.spin()

    def _callback_ball(self, ballsinfo):
        if not self.caminfo:
            return  # No camaraInfo available

        self.work(ballsinfo)

    def work(self, ballsinfo):
        if len(ballsinfo.candidates) == 0:
            rospy.logerr("No candidates in BallsInImage message")
            return

        ballinfo = ballsinfo.candidates[0]
        p = transf(ballinfo.center.x, ballinfo.center.y + ballinfo.diameter // 2, self.caminfo)

        br = BallRelative()
        br.header.stamp = ballsinfo.header.stamp
        br.header.frame_id = "base_link"

        br.ball_relative.x = p[0]
        br.ball_relative.y = p[1]
        br.ball_relative.z = p[2]

        self.ball_relative_pub.publish(br)

    def _callback_camera_info(self, camerainfo):
        self.caminfo = camerainfo


if __name__ == "__main__":
    TransformBall()
