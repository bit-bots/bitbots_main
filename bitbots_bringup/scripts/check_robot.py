#!/usr/bin/python3

import rospy
import rosnode
import roslaunch
import rospkg
import rostopic
from bitbots_msgs.msg import FootPressure


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


left_pressure = None
right_pressure = None


def pressure_cb(msg, is_left=True):
    global left_pressure
    global right_pressure

    if is_left:
        left_pressure = msg
    else:
        right_pressure = msg


def is_motion_started():
    node_names = rosnode.get_node_names("/")
    started = True
    if not "ros_control" in node_names:
        print("ros control not running")
        started = False
    # todo rest of nodes
    return


def check_pressure(msg, min, max, foot_name):
    if min > msg.left_front > max:
        print(bcolors.WARNING + "  " + foot_name + " left_front out of limits. Min %f, Max %f, Value %f" + bcolors.ENDC,
              min, max, msg.left_front)
    if min > msg.left_back > max:
        print(bcolors.WARNING + "  " + foot_name + " left_back out of limits. Min %f, Max %f, Value %f" + bcolors.ENDC,
              min, max, msg.left_front)
    if min > msg.right_back > max:
        print(bcolors.WARNING + "  " + foot_name + " right_back out of limits. Min %f, Max %f, Value %f" + bcolors.ENDC,
              min, max, msg.left_front)
    if min > msg.right_front > max:
        print(
            bcolors.WARNING + "  " + foot_name + " right_front out of limits. Min %f, Max %f, Value %f" + bcolors.ENDC,
            min, max, msg.left_front)


if __name__ == '__main__':
    print("### This script will check the robot hardware and motions. Please follow the instructions\n")

    # start necessary software
    print("First the motion software will be started. Please hold the robot and press enter.\n")
    input("Press Enter to continue...")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    rospack = rospkg.RosPack()
    launch = roslaunch.parent.ROSLaunchParent(uuid, [rospack.get_path('rospy_tutorials') + "/launch/test_node.launch"])
    launch.start()
    while True:
        if is_motion_started():
            break
        else:
            print("Waiting for software to be started \n")
            rospy.sleep(1)

    # check publishing frequency of imu, servos, pwm, goals, pressure, robot_state
    # the topics which will be checked with minimal publishing rate
    topics_to_check = {"/imu_data/raw": 100, "/imu_data": 100, "/joint_states": 100, "/pwm_states": 100,
                       "/motor_goals": 100, "/robot_state": 100, "/foot_pressure/left": 100, "foot_pressure/right": 100}
    rt = rostopic.ROSTopicHz(-1)
    for topic in topics_to_check.keys():
        msg_class, real_topic, _ = rostopic.get_topic_class(topic)
        rospy.Subscriber(real_topic, msg_class, rt.callback_hz, callback_args=topic, tcp_nodelay=True)
    print("Please wait a few seconds for topics to be evaluated\n")
    rospy.sleep(5)
    print("Topics have been evaluated:\n")
    for topic in topics_to_check.keys():
        rate = rt.get_hz(topic)
        if rate < topics_to_check[topic]:
            print(bcolors.WARNING + "  Topic %s: %f \n" + bcolors.ENDC, topic, rate)
        else:
            print("  Topic %s: %f \n", topic, rate)

    # check pressure values when robot in air
    #todo maybe call zero service
    print("\nWe will check the foot pressure sensors next\n")
    input("Please hold the robot in the air so that the feet don't touch the ground and press enter.")
    left_pressure_sub = rospy.Subscriber("/foot_pressure/left", FootPressure, pressure_cb, callback_args=True,
                                         tcp_nodelay=True)
    right_pressure_sub = rospy.Subscriber("/foot_pressure/right", FootPressure, pressure_cb, callback_args=False,
                                          tcp_nodelay=True)
    rospy.sleep(0.5)
    while (not left_pressure) and (not right_pressure):
        print("Waiting to receive pressure msgs\n")
    print("Pressure messages received\n")
    check_pressure(left_pressure, -1, 1, "left")
    check_pressure(right_pressure, -1, 1, "right")

    # check pressure values when robot in walkready
    input("Please put the robot standing on the ground and press enter")
    check_pressure(left_pressure, 20, 40, "left")
    check_pressure(right_pressure, 20, 40, "right")


    # check walk motion

    # check kick motion

    # check stand up front

    # check stand up back

    # shutdown the launch
    launch.shutdown()
