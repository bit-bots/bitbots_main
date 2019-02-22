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

DELAY_TOKEN = "SEND_DELAY"



class LiveToolSender():

    def __init__(self):

        self.send_delay = 500

        self.positionMsg = PositionMsg()
        self.statusMsg = StatusMsg()
        self.detectionMsg = DetectionMsg()
        self.trajectoryMsg = TrajectoryMsg()

        self.udp_ip = rospy.get_param("/live_tool/ip")
        self.udp_port = rospy.get_param("/live_tool/port")

        self.listener()

    # Callbacks
    def callback_ball_location(self,ballRel):
        #print(ballRel)
        self.detectionMsg.setBallRelative(ballRel)
        #print (yaml.load(detectionMsg.getMsg()))
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", ballRel)


    def callback_amcl_pose(self,data):
        self.positionMsg.setPoseWithCovarianceStamped(data)

    def callback_gamestate(self,data):
        self.statusMsg.setGameState(data)
        #print(statusMsg.data)

    def callback_strategy(self, data):
        self.statusMsg.setStrategy(data)

    def callback_obstacles_relative(self,data):
        self.detectionMsg.setObstacleRelative(data)
        #print (yaml.load(detectionMsg.getMsg()))


    #def callback_goal_relative(data):
    #    detectionMsg.setGoalRelative(data)
        #print (yaml.load(detectionMsg.getMsg()))


    #def callback_trajectory(data):
    #    trajectoryMsg.setTrajectory(data)
        #print (yaml.load(trajectoryMsg.getMsg()))

    def callBack_cmd_vel(self,data):
        self.trajectoryMsg.setCmdVel(data)


    def callBack_move_base_simple(self,data):
        self.trajectoryMsg.setMoveBase(data)


    def callback_robot_state(self,data):
        self.statusMsg.setStatusMsg(data)

    # Listens to subscripted messages and send them to the Live tool
    def listener(self):
        # Initializes the Node
        rospy.init_node('udp_listener', anonymous=False)

        # Subscriptions
        rospy.Subscriber("/ball_relative", BallRelative, self.callback_ball_location)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_amcl_pose)
        rospy.Subscriber("/obstacles_relative", ObstaclesRelative, self.callback_obstacles_relative)
        #rospy.Subscriber("/goal_relative", GoalRelative, callback_goal_relative)
        rospy.Subscriber("/gamestate", GameState, self.callback_gamestate)
        rospy.Subscriber("/strategy", Strategy, self.callback_strategy)
        rospy.Subscriber("/robot_state", RobotControlState, self.callback_robot_state)

        rospy.Subscriber("/cmd_vel", Twist, self.callBack_cmd_vel)
        rospy.Subscriber("/move_base_simple/goal", Pose2D, self.callBack_move_base_simple)
        # read ip configurations
        self.readIpConfig()

        # Starts process for Sending UDP-packaged
        p = mp.Process(target=self.worker_thread())
        p.daemon = True
        p.start()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


    # Reads the necessary ip configurations from ip_config.yaml located in resource dir
    def readIpConfig(self):
        rp = rospkg.RosPack()
        ip_filename = os.path.join(rp.get_path('bitbots_live_tool_rqt'), 'resource', 'ip_config.yaml')

        with open(ip_filename, "r") as file:
            global udp_ip, udp_port, send_delay
            ip_config = yaml.load(file)

            self.send_delay = float(ip_config.get(DELAY_TOKEN)) / 1000.0
            
            print("I am sending to: \nIp: "+self.udp_ip+", Port: "+str(self.udp_port)+", Delay: "+str(self.send_delay))

        file.close()


    # send the data every <code>send_delay</code> seconds
    def worker_thread(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while not rospy.is_shutdown():

            sock.sendto( self.positionMsg.getMsg() , (self.udp_ip, self.udp_port))
            sock.sendto( self.statusMsg.getMsg() , (self.udp_ip, self.udp_port))
            sock.sendto( self.trajectoryMsg.getMsg() , (self.udp_ip, self.udp_port))
            sock.sendto( self.detectionMsg.getMsg() , (self.udp_ip, self.udp_port))

            time.sleep(self.send_delay)


if __name__ == '__main__':
    sender = LiveToolSender()
