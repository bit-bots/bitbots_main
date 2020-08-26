#!/usr/bin/env python3
import rospy
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from humanoid_league_msgs.msg import LineInformationRelative, LineIntersectionRelative, GoalRelative
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
import numpy as np

rospy.init_node('feature_tf_listener')

tfBuffer = tf2_ros.Buffer(rospy.Duration(30))
listener = tf2_ros.TransformListener(tfBuffer)

cam_info = None

pubLineContainer = None

message_container = LineInformationRelative()

use_camerainfo = True

def add_corners():
    global tf_buffer
    global cam_info
    global pubLineContainer

    corners = ["cornerTL", "cornerBL", "cornerGTL", "cornerGBL", "cornerTR", "cornerBR", "cornerGTR", "cornerGBR"]

    for corner in corners:
        try:
            trans = tfBuffer.lookup_transform("camera_optical_frame", str(corner), rospy.Time())
            corner_stamped = PoseStamped()

            corner_stamped.header.stamp = rospy.Time.now()
            corner_stamped.header.frame_id = "camera_optical_frame"
            corner_stamped.pose.position.x = trans.transform.translation.x
            corner_stamped.pose.position.y = trans.transform.translation.y
            corner_stamped.pose.position.z = trans.transform.translation.z
            corner_stamped.pose.orientation.w = trans.transform.rotation.w
            corner_stamped.pose.orientation.x = trans.transform.rotation.x
            corner_stamped.pose.orientation.y = trans.transform.rotation.x
            corner_stamped.pose.orientation.z = trans.transform.rotation.z

            #corner_stamped = tfBuffer.transform(corner_stamped, "camera_optical_frame", timeout=rospy.Duration(0.5))

            p = [corner_stamped.pose.position.x, corner_stamped.pose.position.y, corner_stamped.pose.position.z]

            if p[2] > 0:

                k = np.reshape(cam_info.K, (3, 3))
                p_pixel = np.matmul(k, p)
                p_pixel = p_pixel * (1 / p_pixel[2])

                if p_pixel[0] > 0 and p_pixel[0] <= cam_info.width and p_pixel[1] > 0 and p_pixel[1] <= cam_info.height:
                    if np.random.randint(10) < 8 or True:
                        pose = tfBuffer.transform(corner_stamped, "base_footprint", timeout=rospy.Duration(0.5))

                        point_msg = Point()
                        point_msg.x = add_gaussian_noise(pose.pose.position.x)
                        point_msg.y = add_gaussian_noise(pose.pose.position.y)
                        point_msg.z = pose.pose.position.z  # todo set to 0?

                        pixel_msg = LineIntersectionRelative()
                        pixel_msg.pose.pose.pose.position = point_msg
                        pixels.type = pixel_msg.L

                        message_container.intersections.append(pixel_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    message_container.header.stamp = rospy.Time.now()


def add_tcrossings():

    global tfBuffer
    global cam_info

    tcrossings = ["tcrossingTL", "tcrossingBL", "tcrossingTM", "tcrossingBM", "tcrossingTR", "tcrossingBR"]

    for corner in tcrossings:
        try:
            trans = tfBuffer.lookup_transform("camera_optical_frame", str(corner), rospy.Time())
            corner_stamped = PoseStamped()

            corner_stamped.header.stamp = rospy.Time.now()
            corner_stamped.header.frame_id = "camera_optical_frame"
            corner_stamped.pose.position.x = trans.transform.translation.x
            corner_stamped.pose.position.y = trans.transform.translation.y
            corner_stamped.pose.position.z = trans.transform.translation.z
            corner_stamped.pose.orientation.w = trans.transform.rotation.w
            corner_stamped.pose.orientation.x = trans.transform.rotation.x
            corner_stamped.pose.orientation.y = trans.transform.rotation.x
            corner_stamped.pose.orientation.z = trans.transform.rotation.z

            p = [corner_stamped.pose.position.x, corner_stamped.pose.position.y, corner_stamped.pose.position.z]

            if p[2] > 0:

                k = np.reshape(cam_info.K, (3, 3))
                p_pixel = np.matmul(k, p)
                p_pixel = p_pixel * (1/p_pixel[2])

                if p_pixel[0] > 0 and p_pixel[0] <= cam_info.width and p_pixel[1] > 0 and p_pixel[1] <= cam_info.height:
                    if np.random.randint(10) < 8 or True:

                        pose = tfBuffer.transform(corner_stamped, "base_footprint", timeout=rospy.Duration(0.5))

                        point_msg = Point()
                        point_msg.x = add_gaussian_noise(pose.pose.position.x)
                        point_msg.y = add_gaussian_noise(pose.pose.position.y)
                        point_msg.z = pose.pose.position.z#todo set to 0?

                        pixel_msg = LineIntersectionRelative()
                        pixel_msg.pose.pose.pose.position = point_msg
                        pixels.type = pixel_msg.L

                        message_container.intersections.append(pixel_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    message_container.header.stamp = rospy.Time.now()


def add_crosses():
    global tf_buffer
    global cam_info

    crosses = ["crossL", "crossR", "crossTM", "crosMM", "crossBM"]

    for cross in crosses:
        try:
            trans = tfBuffer.lookup_transform("camera_optical_frame", str(cross), rospy.Time(0))

            corner_stamped = PoseStamped()
            corner_stamped.header.stamp = rospy.Time.now()
            corner_stamped.header.frame_id = "camera_optical_frame"
            corner_stamped.pose.position.x = trans.transform.translation.x
            corner_stamped.pose.position.y = trans.transform.translation.y
            corner_stamped.pose.position.z = trans.transform.translation.z
            corner_stamped.pose.orientation = trans.transform.rotation

            p = [corner_stamped.pose.position.x, corner_stamped.pose.position.y, corner_stamped.pose.position.z]
            k = np.reshape(cam_info.K, (3, 3))
            p_pixel = np.matmul(k, p)

            if p_pixel[2] > 0:
                p_pixel = p_pixel * (1/p_pixel[2])

                if p_pixel[0] > 0 and p_pixel[0] <= cam_info.width and p_pixel[1] > 0 and p_pixel[1] <= cam_info.height:
                    if np.random.uniform(10) < 8 or True:
                        pose = tfBuffer.transform(corner_stamped, "base_footprint", timeout=rospy.Duration(0.5))

                        point_msg = Point()
                        point_msg.x = add_gaussian_noise(pose.pose.position.x)
                        point_msg.y = add_gaussian_noise(pose.pose.position.y)
                        point_msg.z = pose.pose.position.z #todo set to 0?

                        pixel_msg = LineIntersectionRelative()
                        pixel_msg.pose.pose.pose.position = point_msg
                        pixels.type = pixel_msg.L

                        message_container.intersections.append(pixel_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    message_container.header.stamp = rospy.Time.now()

def publish_goals():
    global tf_buffer
    global cam_info
    global pubGoals #todo really necessary?!
    detected_goals = []

    goals_msg = GoalRelative()
    goals = ["goalLB", "goalLT", "goalRB", "goalRT"]


    for goal in goals:
        try:
            trans = tfBuffer.lookup_transform("camera_optical_frame", str(goal), rospy.Time(0))

            corner_stamped = PoseStamped()
            corner_stamped.header.frame_id = "camera_optical_frame"
            corner_stamped.pose.position.x = trans.transform.translation.x
            corner_stamped.pose.position.y = trans.transform.translation.y
            corner_stamped.pose.position.z = trans.transform.translation.z
            corner_stamped.pose.orientation = trans.transform.rotation

            p = [corner_stamped.pose.position.x, corner_stamped.pose.position.y, corner_stamped.pose.position.z]
            k = np.reshape(cam_info.K, (3, 3))
            p_pixel = np.matmul(k, p)

            if p_pixel[2] > 0:
                p_pixel = p_pixel * (1/p_pixel[2])

                if p_pixel[0] > 0 and p_pixel[0] <= cam_info.width and p_pixel[1] > 0 and p_pixel[1] <= cam_info.height:
                    if np.random.uniform(10) < 8 or True:
                        pose = tfBuffer.transform(corner_stamped, "base_footprint", timeout=rospy.Duration(0.5))

                        point_msg = Point()
                        point_msg.x = add_gaussian_noise(pose.pose.position.x)
                        point_msg.y = add_gaussian_noise(pose.pose.position.y)
                        point_msg.z = pose.pose.position.z #todo set to 0?

                        detected_goals.append(point_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    if len(detected_goals) == 1:
        goals_msg.left_post = detected_goals[0]
        goals_msg.right_post = detected_goals[0]

    elif len(detected_goals) == 2:
        goals_msg.left_post = detected_goals[0]
        goals_msg.right_post = detected_goals[1]

    goals_msg.header.stamp = rospy.Time.now()

    pubGoals.publish(goals_msg)

def add_gaussian_noise(x):
    return x


def cam_info_cb(msg):
    global cam_info
    cam_info = msg
    global tf_buffer
    global pubLineContainer

    add_corners()
    add_tcrossings()
    add_crosses()
    publish_goals()

    pubGoals.publish(message_container)
    message_container = LineInformationRelative()

if __name__ == '__main__':
    cam_info_sub = rospy.Subscriber("/camera_info", CameraInfo, cam_info_cb)

    pubLineContainer = rospy.Publisher('/lines', LineInformationRelative, queue_size=10)

    pubGoals = rospy.Publisher('/goals_simulated', GoalRelative, queue_size=10)

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn(
                "We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
        except rospy.exceptions.ROSInterruptException:
            exit()





