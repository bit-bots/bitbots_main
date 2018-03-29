#!/usr/bin/env python2.7
import rospy
from humanoid_league_msgs.msg import BallRelative, BallInImage, BallsInImage
from sensor_msgs.msg import CameraInfo
import tf2_ros
import math
from tf2_geometry_msgs import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np
import tf
import tf_conversions


class TransformBall(object):
    def __init__(self):
        #rospy.Subscriber("ball_in_image", BallsInImage, self._callback_ball, queue_size=1)
        rospy.Subscriber("/minibot/camera/camera_info", CameraInfo, self._callback_camera_info)
        self.marker_pub = rospy.Publisher("ballpoint", Marker, queue_size=10)
        self.ball_relative_pub = rospy.Publisher("ball_relative", BallRelative, queue_size=10)

        self.caminfo = None  # type:CameraInfo

        rospy.init_node("transform_ball")

        object_height = 0.1

        tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(3)
        
        while not rospy.is_shutdown():
            while not self.caminfo:
                print("waiting for camera info")
                rospy.sleep(2)

            x = int(raw_input("x:"))
            y = int(raw_input("y:"))

            br = BallRelative()
            camera_pose = tf_buffer.lookup_transform("base_link", "L_CAMERA", rospy.get_rostime()-rospy.Duration(0.2), rospy.Duration(1.0))

            br.header.stamp = rospy.get_rostime()-rospy.Duration(0.2)#todo ball in image time!
            br.header.frame_id = "L_CAMERA"

            # normalized camera coordinates coordinate system
            #  -1,0 ------------- 1,1
            #       |           |
            #       |    0,0    |
            #       |           |
            # -1,-1 ------------- 1,0
            normalized_x = 2 * (float(x)-(float(self.caminfo.width)/2))/float(self.caminfo.width)
            normalized_y = (-2 * (float(y)-(float(self.caminfo.height)/2))/float(self.caminfo.height)) * (float(self.caminfo.height)/float(self.caminfo.width))
            print self.caminfo.width
            print(normalized_x)
            print(normalized_y)
            fov = 1.012300
            focal_length = 1/math.tan(fov/2 )

            point_on_image = PoseStamped()
            point_on_image.header.frame_id = "L_CAMERA"
            point_on_image.header.stamp = rospy.get_rostime() - rospy.Duration(0.2)
            point_on_image.pose.position.x = focal_length
            point_on_image.pose.position.y = -normalized_x
            point_on_image.pose.position.z = normalized_y

            msg = Marker()
            msg.header.frame_id = "L_CAMERA"
            msg.header.stamp = rospy.get_rostime() - rospy.Duration(0.2)
            msg.pose.position.x = 0
            msg.pose.position.y = 0
            msg.pose.position.z = 0



            vec = [point_on_image.pose.position.x, point_on_image.pose.position.y, point_on_image.pose.position.z ** 2]
            length = np.linalg.norm(vec)
            quaternion = tf_conversions.transformations.quaternion_from_euler(0,
                                                                              math.acos(np.dot(vec, [0, 1, 0])/length),
                                                                              math.acos(np.dot(vec, [0, 0, 1])/length))
            msg.pose.orientation.x = quaternion[0]
            msg.pose.orientation.y = quaternion[1]
            msg.pose.orientation.z = quaternion[2]
            msg.pose.orientation.w = quaternion[3]
            msg.type = Marker.ARROW
            msg.lifetime = rospy.Duration(100)
            msg.action = 0
            msg.ns = "ball_finder"
            msg.id = 1
            msg.scale.x = length
            msg.scale.y = 0.05
            msg.scale.z = 0.05
            msg.color.r = 1
            msg.color.g = 0
            msg.color.b = 0
            msg.color.a = 1

            self.marker_pub.publish(msg)

            msg.pose.orientation.x = 0
            msg.pose.orientation.y = 0
            msg.pose.orientation.z = 0
            msg.pose.orientation.w = 1
            msg.pose.position = point_on_image.pose.position

            msg.type = Marker.SPHERE
            msg.id = 2
            msg.color.g = 1
            msg.scale.x = 0.05
            msg.scale.y = 0.05
            msg.scale.z = 0.05

            self.marker_pub.publish(msg)

            field_normal = PoseStamped()
            field_normal.header.frame_id = "base_link"
            field_normal.header.stamp = rospy.get_rostime() - rospy.Duration(0.2)
            field_normal.pose.position.x = 0
            field_normal.pose.position.y = 0
            field_normal.pose.position.z = 1
            
            field_point = PoseStamped()
            field_point.header.frame_id = "base_link"
            field_point.header.stamp = rospy.get_rostime() - rospy.Duration(0.2)
            field_point.pose.position.x = 0
            field_point.pose.position.y = 0
            field_point.pose.position.z = object_height

            field_normal = tf_buffer.transform(field_normal,"L_CAMERA")
            field_point = tf_buffer.transform(field_point,"L_CAMERA")

            msg.type = Marker.CUBE
            msg.id = 3
            msg.color.b = 1
            msg.scale.x = 0.05
            msg.scale.y = 0.05
            msg.scale.z = 0.05
            msg.pose.position = field_point.pose.position

            print field_normal.header.frame_id
            self.marker_pub.publish(msg)

            msg.pose.position = field_normal.pose.position

            msg.id = 4
            self.marker_pub.publish(msg)

            p = self.linePlaneCollision(np.array([field_normal.pose.position.x, field_normal.pose.position.y, field_normal.pose.position.z]),
                                        np.array([field_point.pose.position.x, field_point.pose.position.y, field_point.pose.position.z]),
                                        np.array([point_on_image.pose.position.x, point_on_image.pose.position.y, point_on_image.pose.position.z]),
                                        np.array([0, 0, 0]))
            print(len(p))

            if len(p):
                br.ball_relative.x = p[0]
                br.ball_relative.y = p[1]
                br.ball_relative.z = p[2]
                br.header.frame_id = "L_CAMERA"
                br.header.stamp = rospy.get_rostime() - rospy.Duration(0.2)
                br.confidence = 1.
                self.ball_relative_pub.publish(br)


    def _callback_camera_info(self, camerainfo):
        self.caminfo = camerainfo

    def linePlaneCollision(self, planeNormal, planePoint, rayDirection, rayPoint, epsilon=1e-6):

        ndotu = planeNormal.dot(rayDirection)
        if abs(ndotu) < epsilon:
            return []

        w = rayPoint - planePoint
        si = -planeNormal.dot(w) / ndotu
        Psi = w + si * rayDirection + planePoint
        return Psi


if __name__ == "__main__":
    TransformBall()
