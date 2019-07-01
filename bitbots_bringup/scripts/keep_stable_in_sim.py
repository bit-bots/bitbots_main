#!/usr/bin/env python2.7

import rospy
from bitbots_msgs.msg import FootPressure
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest

import tf


position = None
roll = None
pitch = None
yaw = None


def state_update(state_msg):
    global position
    global roll
    global pitch
    global yaw

    index = 0
    for name in state_msg.name:
        if name == "/":
            break
        index += 1

    position = state_msg.pose[index].position
    orientation = state_msg.pose[index].orientation
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]


if __name__ == "__main__":
    rospy.init_node("keep_stable_sim")
    rospy.wait_for_service("/gazebo/set_model_state")
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)


    goal_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, state_update, tcp_nodelay=True)

    request = SetModelStateRequest()
    request.model_state.model_name = "/"
    # wait because we want to be called
    rospy.sleep(1.0)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            # check if we have values already, otherwise we will do math with none
            if(yaw):
                request.model_state.pose.position = position
                quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
                request.model_state.pose.orientation.x = quaternion[0]
                request.model_state.pose.orientation.y = quaternion[1]
                request.model_state.pose.orientation.z = quaternion[2]
                request.model_state.pose.orientation.w = quaternion[3]
                set_state(request)
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn(
            "We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
        except rospy.exceptions.ROSInterruptException:
            exit()
