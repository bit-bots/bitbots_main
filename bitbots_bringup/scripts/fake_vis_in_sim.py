#!/usr/bin/env python2.7

from gazebo_msgs.msg import ModelStates, ModelState
import tf2_ros

tf_buffer = tf2_ros.tf_buffer

model_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, state_update, tcp_nodelay=True)

ball_pose = None
goal1_pose = None
goal2_pose = None

def state_update(state_msg):
    global ball_pose
    global goal1_pose
    global goal2_pose

    ball_index = 0
    for name in state_msg.name:
        if name == "ball":
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
    goal1_pose = state_msg.pose[goal1_index]
    goal2_pose = state_msg.pose[goal2_index]



if __name__ == "__main__":
    rospy.init_node("fake_vis_in_sim")

    goal_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, state_update, tcp_nodelay=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:

            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rospy.logwarn(
            "We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
        except rospy.exceptions.ROSInterruptException:
            exit()
