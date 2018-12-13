#!/usr/bin/env python
import rospy
import rospkg
import os
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Pose2D

from humanoid_league_msgs.msg import BallRelative, ObstaclesRelative, GoalRelative, GameState, Strategy, RobotControlState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import time
import multiprocessing as mp
import yaml
import socket

from detection_msg import DetectionMsg
from position_msg import PositionMsg
from status_msg import StatusMsg
from trajectory_msg import TrajectoryMsg

SEND_TOKEN = "IP_SEND_TO"
PORT_TOKEN = "UDP_PORT"
DELAY_TOKEN = "SEND_DELAY"

udp_ip = ""
udp_port = 5005
send_delay = 500

positionMsg = PositionMsg()
statusMsg = StatusMsg()
detectionMsg = DetectionMsg()
trajectoryMsg = TrajectoryMsg()


# Callbacks
def callback_ball_location(ballRel):
    #print(ballRel)
    detectionMsg.setBallRelative(ballRel)
    #print (yaml.load(detectionMsg.getMsg()))
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", ballRel)


def callback_amcl_pose(data):
    positionMsg.setPoseWithCovarianceStamped(data)

def callback_gamestate(data):
    statusMsg.setGameState(data)
    #print(statusMsg.data)

def callback_strategy(data):
    statusMsg.setStrategy(data)

def callback_obstacles_relative(data):
    detectionMsg.setObstacleRelative(data)
    #print (yaml.load(detectionMsg.getMsg()))


#def callback_goal_relative(data):
#    detectionMsg.setGoalRelative(data)
    #print (yaml.load(detectionMsg.getMsg()))


#def callback_trajectory(data):
#    trajectoryMsg.setTrajectory(data)
    #print (yaml.load(trajectoryMsg.getMsg()))

def callBack_cmd_vel(data):
    trajectoryMsg.setCmdVel(data)


def callBack_move_base_simple(data):
    trajectoryMsg.setMoveBase(data)


def callback_robot_state(data):
    statusMsg.setStatusMsg(data)

# Listens to subscripted messages and send them to the Live tool
def listener():
    # Initializes the Node
    rospy.init_node('udp_listener', anonymous=False)

    # Subscriptions
    rospy.Subscriber("/ball_relative", BallRelative, callback_ball_location)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback_amcl_pose)
    rospy.Subscriber("/obstacles_relative", ObstaclesRelative, callback_obstacles_relative)
    #rospy.Subscriber("/goal_relative", GoalRelative, callback_goal_relative)
    rospy.Subscriber("/gamestate", GameState, callback_gamestate)
    rospy.Subscriber("/strategy", Strategy, callback_strategy)
    rospy.Subscriber("/robot_state", RobotControlState, callback_robot_state)

    rospy.Subscriber("/cmd_vel", Twist, callBack_cmd_vel)
    rospy.Subscriber("/move_base_simple/goal", Pose2D, callBack_move_base_simple)
    # read ip configurations
    readIpConfig()

    # Starts process for Sending UDP-packaged
    p = mp.Process(target=worker_thread())
    p.daemon = True
    p.start()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# Reads the necessary ip configurations from ip_config.yaml located in resource dir
def readIpConfig():
    rp = rospkg.RosPack()
    ip_filename = os.path.join(rp.get_path('rqt_live_tool'), 'resource', 'ip_config.yaml')

    with open(ip_filename, "r") as file:
        global udp_ip, udp_port, send_delay
        ip_config = yaml.load(file)

        udp_ip = ip_config.get(SEND_TOKEN)
        udp_port = int(ip_config.get(PORT_TOKEN))
        send_delay = float(ip_config.get(DELAY_TOKEN)) / 1000.0

        print(udp_ip, udp_port, send_delay)

    file.close()


# send the data every <code>send_delay</code> seconds
def worker_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while not rospy.is_shutdown():

        sock.sendto( positionMsg.getMsg() , (udp_ip, udp_port))
        sock.sendto( statusMsg.getMsg() , (udp_ip, udp_port))
        sock.sendto( trajectoryMsg.getMsg() , (udp_ip, udp_port))
        sock.sendto( detectionMsg.getMsg() , (udp_ip, udp_port))

        time.sleep(send_delay)


if __name__ == '__main__':
    listener()
