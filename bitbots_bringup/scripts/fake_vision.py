#!/usr/bin/env python2.7

from gazebo_msgs.msg import ModelStates, ModelState
import tf2_ros
import rospy
from sensor_msgs.msg import CameraInfo
from tf2_geometry_msgs import PoseStamped
import numpy as np
from humanoid_league_msgs.msg import BallRelative, GoalRelative
from copy import deepcopy

rospy.init_node("fake_vis_in_sim")

tf_buffer = tf2_ros.Buffer(rospy.Duration(30))
tf_listener = tf2_ros.TransformListener(tf_buffer)


ball_pose = None
goal1_pose = None
goal2_pose = None
cam_info = None
ball_pub = None
goal_pub = None

def state_update(state_msg):
    global ball_pose
    global goal1_pose
    global goal2_pose
    global tf_buffer
    global cam_info
    global ball_pub
    global goal_pub

    if not cam_info:
        return

    ball_index = 0
    for name in state_msg.name:
        if name == "teensize_ball":
            break
        ball_index += 1


    goal_index1 = 0
    goal_index2 = 0
    for name in state_msg.name:
        if name == "teensize_goal":
            if goal_index1 == 0:
                goal_index1 = goal_index2
            else:
                break
        goal_index2 += 1

    ball_pose = state_msg.pose[ball_index]
    goal1_pose = state_msg.pose[goal_index1]
    goal2_pose = state_msg.pose[goal_index2]

    ball_pose_stamped = PoseStamped()
    ball_pose_stamped.header.stamp = rospy.Time.now()
    ball_pose_stamped.header.frame_id = "map"
    ball_pose_stamped.pose = ball_pose
    ball_in_camera_optical_frame = tf_buffer.transform(ball_pose_stamped, cam_info.header.frame_id, timeout=rospy.Duration(0.5))

    # we only have to compute if the ball is inside the image, if the ball is in front of the camera
    if ball_in_camera_optical_frame.pose.position.z >= 0:
        p = [ball_in_camera_optical_frame.pose.position.x, ball_in_camera_optical_frame.pose.position.y, ball_in_camera_optical_frame.pose.position.z]
        k = np.reshape(cam_info.K, (3,3))
        p_pixel = np.matmul(k, p)
        p_pixel = p_pixel * (1/p_pixel[2])

        # make sure that the transformed pixel is inside the resolution and positive.
        # also z has to be positive to make sure that the ball is in front of the camera and not in the back
        if 0 < p_pixel[0] <= cam_info.width and 0 < p_pixel[1] <= cam_info.height:
            ball_relative = BallRelative()
            ball_relative.header.stamp = ball_in_camera_optical_frame.header.stamp
            ball_relative.header.frame_id = "base_footprint"
            ball_in_footprint_frame = tf_buffer.transform(ball_in_camera_optical_frame, "base_footprint",
                                                    timeout=rospy.Duration(0.5))
            ball_relative.header = ball_in_footprint_frame.header
            ball_relative.ball_relative = ball_in_footprint_frame.pose.position
            ball_relative.confidence = 1.0
            ball_pub.publish(ball_relative)
    # todo make this also listen to interactive markers

    for gp in (goal1_pose, goal2_pose):
        goal_pose_stamped = PoseStamped()
        goal_pose_stamped.header.stamp = rospy.Time.now()
        goal_pose_stamped.header.frame_id = "map"
        goal_pose_stamped.pose = gp

        goal = GoalRelative()
        goal.header.stamp = goal_pose_stamped.header.stamp
        goal.header.frame_id = "base_footprint"

        left_post = deepcopy(goal_pose_stamped)
        left_post.pose.position.y += 1.35
        left_post_in_camera_optical_frame = tf_buffer.transform(left_post, cam_info.header.frame_id, timeout=rospy.Duration(0.5))

        lpost_visible = False
        rpost_visible = False

        if left_post_in_camera_optical_frame.pose.position.z >= 0:
            p = [left_post_in_camera_optical_frame.pose.position.x, left_post_in_camera_optical_frame.pose.position.y, left_post_in_camera_optical_frame.pose.position.z]
            k = np.reshape(cam_info.K, (3, 3))
            p_pixel = np.matmul(k, p)
            p_pixel = p_pixel * (1 / p_pixel[2])

            if 0 < p_pixel[0] <= cam_info.width and 0 < p_pixel[1] <= cam_info.height:
                goal.left_post = tf_buffer.transform(left_post, "base_footprint",
                                                    timeout=rospy.Duration(0.5)).pose.position
                lpost_visible = True

        right_post = goal_pose_stamped
        right_post.pose.position.y -= 1.35
        right_post_in_camera_optical_frame = tf_buffer.transform(right_post, cam_info.header.frame_id, timeout=rospy.Duration(0.5))
        if right_post_in_camera_optical_frame.pose.position.z >= 0:
            p = [right_post_in_camera_optical_frame.pose.position.x, right_post_in_camera_optical_frame.pose.position.y, right_post_in_camera_optical_frame.pose.position.z]
            k = np.reshape(cam_info.K, (3, 3))
            p_pixel = np.matmul(k, p)
            p_pixel = p_pixel * (1 / p_pixel[2])

            if 0 < p_pixel[0] <= cam_info.width and 0 < p_pixel[1] <= cam_info.height:
                goal.right_post = tf_buffer.transform(right_post_in_camera_optical_frame, "base_footprint",
                                                    timeout=rospy.Duration(0.5)).pose.position
                rpost_visible = True

        if rpost_visible or lpost_visible:
            if not lpost_visible:
                goal.left_post = goal.right_post
            elif not rpost_visible:
                goal.right_post = goal.left_post
            goal.confidence = 1
            goal_pub.publish(goal)


def cam_info_cb(msg):
    global cam_info
    cam_info = msg


if __name__ == "__main__":
    # wait for transforms to become available
    tf_buffer.can_transform("base_footprint", "camera_optical_frame", rospy.Time(0), timeout=rospy.Duration(30))
    tf_buffer.can_transform("map", "camera_optical_frame", rospy.Time(0), timeout=rospy.Duration(30))

    ball_pub = rospy.Publisher("/ball_relative", BallRelative, queue_size=1)
    goal_pub = rospy.Publisher("/goal_relative", GoalRelative, queue_size=1)
    model_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, state_update, tcp_nodelay=True, queue_size=1)
    cam_info_sub = rospy.Subscriber("/camera_info", CameraInfo, cam_info_cb)

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn(
            "We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
        except rospy.exceptions.ROSInterruptException:
            exit()
