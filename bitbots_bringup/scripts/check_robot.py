#!/usr/bin/python3

import rospy
import rosnode
import roslaunch
import rospkg
import rostopic
from bitbots_msgs.msg import FootPressure
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import actionlib
from bitbots_msgs.msg import KickGoal, KickAction, KickFeedback
from geometry_msgs.msg import Vector3, Quaternion
from tf.transformations import quaternion_from_euler


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def print_warn(str):
    print(bcolors.WARNING + str + bcolors.ENDC)


diag_status = None


def diagnostic_cb(msg: DiagnosticStatus):
    global diag_status
    diag_status = msg.level


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
    nodes_in_motion = {"/ros_control", "/hcm", "/walking", "/animation", "/dynamic_kick"}
    for node in nodes_in_motion:
        if not node in node_names:
            print(F"{node} not running")
            started = False
    return started


def check_pressure(msg, min, max, foot_name):
    okay = True
    if msg.left_front < min or msg.left_front > max:
        print_warn(F"  {foot_name} left_front out of limits. Min {min} Max {max} Value {round(msg.left_front, 2)}\n")
        okay = False
    if msg.left_back < min or msg.left_back > max:
        print_warn(F"  {foot_name} left_back out of limits. Min {min} Max {max} Value {round(msg.left_back, 2)}\n")
        okay = False
    if msg.right_back < min or msg.right_back > max:
        print_warn(F"  {foot_name} right_back out of limits. Min {min} Max {max} Value {round(msg.right_back, 2)}\n")
        okay = False
    if msg.right_front < min or msg.right_front > max:
        print_warn(F"  {foot_name} right_front out of limits. Min {min} Max {max} Value {round(msg.right_front, 2)}\n")
        okay = False
    return okay


if __name__ == '__main__':
    print("### This script will check the robot hardware and motions. Please follow the instructions\n")

    rospy.init_node("checker")

    # start necessary software
    print("First the motion software will be started. Please hold the robot and press enter.\n")
    input("Press Enter to continue...")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    rospack = rospkg.RosPack()
    launch = roslaunch.parent.ROSLaunchParent(uuid, [
        rospack.get_path('bitbots_bringup') + "/launch/motion_standalone.launch"])
    launch.start()
    while True:
        if is_motion_started():
            rospy.sleep(5)
            break
        else:
            print("Waiting for software to be started \n")
            rospy.sleep(1)
    print("\n\n")

    # check diagnostic status
    print("Will check diagnostic status of robot.\n")
    diag_sub = rospy.Subscriber("/diagnostics_toplevel_state", DiagnosticStatus, diagnostic_cb, tcp_nodelay=True)
    rospy.sleep(3)
    if diag_status == DiagnosticStatus.OK:
        print("    Diagnostic status okay.")
    else:
        print_warn("    Diagnostics report problem. Please use rqt_monitor to investigate.")
    print("\n\n")

    # check publishing frequency of imu, servos, pwm, goals, pressure, robot_state
    # the topics which will be checked with minimal publishing rate
    topics_to_check = {"/imu/data": 900, "/joint_states": 900, "/robot_state": 900, "/foot_pressure_left/raw": 900,
                       "/foot_pressure_left/filtered": 900, "/foot_pressure_right/raw": 900,
                       "/foot_pressure_right/filtered": 900, "/core/vdxl": 9, "/diagnostics_toplevel_state": 9}
    rts = []
    for topic in topics_to_check.keys():
        msg_class, real_topic, _ = rostopic.get_topic_class(topic)
        if real_topic is None or msg_class is None:
            print_warn(F"Problem with topic {topic}")
        else:
            rt = rostopic.ROSTopicHz(-1)
            rt_sub = rospy.Subscriber(topic, msg_class, rt.callback_hz, callback_args=topic, tcp_nodelay=True)
            rts.append((rt, rt_sub))
    print("Please wait a few seconds for publishing rates of topics to be evaluated\n")
    rospy.sleep(5)
    print("Topics have been evaluated:\n")
    i = 0
    for topic in topics_to_check.keys():
        rate = rts[i][0].get_hz(topic)[0]
        if rate is None or rate < topics_to_check[topic]:
            print_warn(F"  Low rate on Topic {topic}: {round(rate, 2)} \n")
        else:
            print(F"  Okay rate Topic {topic}: {round(rate, 2)} \n")
        i += 1

    # check pressure values when robot in air
    print("\n\n")
    print("We will check the foot pressure sensors next\n")
    input("Please hold the robot in the air so that the feet don't touch the ground and press enter.")
    left_pressure_sub = rospy.Subscriber("/foot_pressure_left/filtered", FootPressure, pressure_cb, callback_args=True,
                                         tcp_nodelay=True)
    right_pressure_sub = rospy.Subscriber("/foot_pressure_right/filtered", FootPressure, pressure_cb,
                                          callback_args=False,
                                          tcp_nodelay=True)
    rospy.sleep(0.5)
    while (not left_pressure) and (not right_pressure):
        rospy.loginfo_throttle(1, "Waiting to receive pressure msgs\n")
    print("Pressure messages received\n")
    both_okay = True
    both_okay = both_okay and check_pressure(left_pressure, -1, 1, "left")
    both_okay = both_okay and check_pressure(right_pressure, -1, 1, "right")
    if not both_okay:
        print_warn("Pressure not correctly zero. Will try call zero service.\n")
        # call zero service
        zero_l = rospy.ServiceProxy("/foot_pressure_left/set_foot_zero", Empty)
        zero_r = rospy.ServiceProxy("/foot_pressure_right/set_foot_zero", Empty)
        zero_l()
        zero_r()
        # wait and check again
        rospy.sleep(1)
        both_okay = True
        both_okay = both_okay and check_pressure(left_pressure, -1, 1, "left")
        both_okay = both_okay and check_pressure(right_pressure, -1, 1, "right")
        if both_okay:
            print("Pressures correct after calling zero service. You can continue normally.\n")
        else:
            print_warn("Pressures still wrong. Please investigate the problem.\n")

    # check pressure values when robot in walkready
    input("Please put the robot standing on the ground and press enter")
    both_okay = True
    both_okay = both_okay and check_pressure(left_pressure, 20, 40, "left")
    both_okay = both_okay and check_pressure(right_pressure, 20, 40, "right")
    if both_okay:
        print("Pressure seems to be okay\n")
    else:
        print_warn("Problem with pressure. "
                   "Please recalibrate the sensors using rosrun bitbots_ros_control pressure_calibaration\n")

    # check fall front
    input("\nNext we will check front falling detection.\n "
          "Please let the robot fall on its belly, but hold it to prevent damage. "
          "The robot should perform a safety motion. Afterwards it should stand up by itself. "
          "Press enter when you're done.\n")

    input("\nNext we will check back falling detection.\n "
          "Please let the robot fall on its back, but hold it to prevent damage. "
          "The robot should perform a safety motion. Afterwards it should stand up by itself. "
          "Press enter when you're done.\n")

    # check walk motion
    walk_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    text = input(
        "\nWe will check walking of the robot. After pressing enter, robot will start walking in different directions. "
        "It will stop when it is finished. Please make sure there is space and catch it if it falls. Press y if you want to check walking.")
    if text == "y":
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.1
        walk_pub.publish(cmd_msg)
        rospy.sleep(5)
        cmd_msg.linear.x = -0.1
        walk_pub.publish(cmd_msg)
        rospy.sleep(5)
        cmd_msg.linear.x = 0.0
        cmd_msg.linear.y = 0.1
        walk_pub.publish(cmd_msg)
        rospy.sleep(5)
        cmd_msg.linear.y = -0.1
        walk_pub.publish(cmd_msg)
        rospy.sleep(5)
        cmd_msg.linear.y = 0
        walk_pub.publish(cmd_msg)
        rospy.sleep(1)

    # check kick motion
    text = input("\nWe will check the kick motion. Please hold make sure the robot is safe. "
                 "Press y if you want to perform this test.")
    client = actionlib.SimpleActionClient('dynamic_kick', KickAction)
    goal = KickGoal()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "base_footprint"
    goal.ball_position.x = 0.2
    goal.ball_position.y = -0.09
    goal.ball_position.z = 0
    goal.kick_direction = Quaternion(*quaternion_from_euler(0, 0, 0))
    goal.kick_speed = 1


    def done_cb(state, result):
        print('Action completed: ', end='')
        if state == GoalStatus.PENDING:
            print('Pending')
        elif state == GoalStatus.ACTIVE:
            print('Active')
        elif state == GoalStatus.PREEMPTED:
            print('Preempted')
        elif state == GoalStatus.SUCCEEDED:
            print('Succeeded')
        elif state == GoalStatus.ABORTED:
            print('Aborted')
        elif state == GoalStatus.REJECTED:
            print('Rejected')
        elif state == GoalStatus.PREEMPTING:
            print('Preempting')
        elif state == GoalStatus.RECALLING:
            print('Recalling')
        elif state == GoalStatus.RECALLED:
            print('Recalled')
        elif state == GoalStatus.LOST:
            print('Lost')
        else:
            print('Unknown state', state)
        print(str(result))

    def active_cb():
        print("Server accepted action")

    def feedback_cb(feedback):
        if len(sys.argv) > 1 and sys.argv[1] == '--feedback':
            print('Feedback')
            print(feedback)
            print()

    client.done_cb = done_cb
    client.feedback_cb = feedback_cb
    client.active_cb = active_cb
    client.send_goal(goal)

    client.wait_for_result()

    input("All tests finished, script will exit and turn off robot. Please hold robot and press enter")
    # shutdown the launch
    launch.shutdown()
