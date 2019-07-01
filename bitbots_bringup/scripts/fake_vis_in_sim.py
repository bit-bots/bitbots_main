#!/usr/bin/env python2.7

from gazebo_msgs.msg import ModelStates, ModelState
import tf2_ros
import rospy
from sensor_msgs.msg import CameraInfo
from tf2_geometry_msgs import PoseStamped
import numpy as np

rospy.init_node("fake_vis_in_sim")

tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
tf_listener = tf2_ros.TransformListener(tf_buffer)


ball_pose = None
goal1_pose = None
goal2_pose = None
cam_info = None

def state_update(state_msg):
    global ball_pose
    global goal1_pose
    global goal2_pose
    global tf_buffer
    global cam_info

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
    rospy.logerr_throttle(1, state_msg.pose[ball_index])
    goal1_pose = state_msg.pose[goal_index1]
    goal2_pose = state_msg.pose[goal_index2]

    ball_pose_stamped = PoseStamped()
    ball_pose_stamped.header.stamp = rospy.Time.now()
    ball_pose_stamped.header.frame_id = "map"
    ball_pose_stamped.pose = ball_pose

    ball_pose_stamped = tf_buffer.transform(ball_pose_stamped, cam_info.header.frame_id, timeout=rospy.Duration(0.1))

    p = [ball_pose_stamped.pose.position.x, ball_pose_stamped.pose.position.y, ball_pose_stamped.pose.position.z]
    k = np.reshape(cam_info.K, (3,3))
    p_pixel = np.matmul(k, p)
    p_pixel = p_pixel * (1/p_pixel[2])
    rospy.logwarn_throttle(1, p_pixel)




def cam_info_cb(msg):
    global cam_info
    cam_info = msg


if __name__ == "__main__":

    model_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, state_update, tcp_nodelay=True)
    cam_info_sub = rospy.Subscriber("/camera_info", CameraInfo, cam_info_cb)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            rospy.spin()
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn(
            "We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
        except rospy.exceptions.ROSInterruptException:
            exit()
