#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

import rospy
import rospkg
import tf.transformations
import os
import time, math

from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState, Imu
from bitbots_buttons.msg import Buttons
from humanoid_league_msgs.msg import RobotControlState
from bitbots_hardware_rqt.msg import BatteryMessage #TODO: fix message and then move to bitbots_msgs
from bitbots_msgs.msg import Cpu

import multiprocessing as mp
import json
import yaml
import socket


class Hardwaretool():
    """Class that collects information from other nodes and stores them in an Robot class object.

    Subscribes: diagnostics, joint_states, buttons, imu/data, robot_state, cpu_info
    """
    def __init__(self):
        """Initializes the object: Creates a node, subscribers and a robot class Object"""
        self.robot = Robot()
        rospy.init_node("hardwaretool")
        self.diagnostics = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnostics_to_object)
        self.joint_states = rospy.Subscriber("/joint_states", JointState, self.joint_states_to_object)
        self.buttons = rospy.Subscriber("/buttons", Buttons, self.buttons_to_object)
        self.imu = rospy.Subscriber("/imu/data", Imu, self.imu_to_object)
        self.state = rospy.Subscriber("/robot_state", RobotControlState, self.robot_state_to_object)
        self.cpu_info = rospy.Subscriber("/cpu_info", Cpu, self.cpu_info_to_object) 
        self.bat_info = rospy.Subscriber("/battery_info", BatteryMessage, self.battery_info_to_object)
        

        self.udp_ip = rospy.get_param("/hardware_tool/rqt_ip")
        self.udp_port = rospy.get_param("/hardware_tool/hardware_rqt_port")
        self.send_delay = rospy.get_param("/hardware_tool/send_delay")

        print("I am sending to: \nIp: "+self.udp_ip+", Port: "+str(self.udp_port)+", Delay: "+str(self.send_delay))

        p = mp.Process(target=self.worker_thread(self.udp_ip, self.udp_port))
        p.daemon = True
        p.start()


        rospy.spin()


    def worker_thread(self, udp_ip,udp_port):
        """Second thread to send messages continuously without blocking everything."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while not rospy.is_shutdown():
            self.robot.timestamp = str(rospy.get_rostime().secs) + '.' + str(rospy.get_rostime().nsecs)
            sock.sendto(json.dumps(self.robot.__dict__), (udp_ip, udp_port))

            time.sleep(self.send_delay)

    def diagnostics_to_object(self, diagnostics):
        """Collects basic information about the motors and stores them in self.robot
        
        Provides motor name, hardware ID, voltage, error byte, temperature and potential error messages for each motor.
        Depends on information from topic /diagnostics.
        """
        for i in diagnostics.status:
            motor_exists = False
            if i.level == 1: #to make sure we only read motor states
                for j in self.robot.motors:
                    if j['name'] == i.values[2].value:
                        j['id'] = i.hardware_id
                        j['input_voltage'] = i.values[1].value
                        j['error_byte'] = i.values[0].value
                        j['temperature'] = i.values[3].value
                        j['message'] = i.message
                        motor_exists = True
                if not motor_exists:
                    self.robot.motors.append({'name':i.values[2].value, 'id':i.hardware_id, 'input_voltage':i.values[1].value,
                    'error_byte':i.values[0].value, 'temperature':i.values[3].value, 'message':i.message, 'torque':'0', 
                    'position':'0'})


    def joint_states_to_object(self, states):
        """Collects information about the motor position and stores them in self.robot
        
        Depends on information from topic /joint_states.
        """
        for i in range(0, len(states.name)):
            for j in range(0, len(self.robot.motors)):
                if self.robot.motors[j]['name'] == states.name[i]:
                    self.robot.motors[j]['position'] = str(math.degrees(states.position[i]))
                    self.robot.motors[j]['torque'] = str(states.effort[i])

    def buttons_to_object(self, buttons):
        """Collects information about the button states and stores them in self.robot

        Depends on information from topic /buttons.
        """
        self.robot.button1 = buttons.button1
        self.robot.button2 = buttons.button2

    def imu_to_object(self, imu_data):
        """Collects information about the robots orientation and stores them in self.robot
        
        Provides robot pitch, yaw and roll
        Depends on information from topic /imu/data
        """
        #quaternion = (imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w)
        #euler = tf.transformations.euler_from_quaternion(quaternion)
        #self.robot.yaw = euler[2]
        #self.robot.pitch = euler[1]       #use these lines for imu orientation instead of velocity     
        #self.robot.roll = euler[0]

        self.robot.yaw =  imu_data.angular_velocity.z
        self.robot.pitch = imu_data.angular_velocity.y
        self.robot.roll = imu_data.angular_velocity.x

    def robot_state_to_object(self, state):
        """Collects and stores the robot state to self.robot
        
        Depends on information from topic /robot_state
        """
        self.robot.state = state.state

    def cpu_info_to_object(self, data):
        """Collects information from multiple CPUs and stores them in self.robot

        Provides temperature and usage of each CPU.
        Depends on information from topic /cpu_info
        """
        cpu_exists = False
        for i in self.robot.cpulist:
            if i['name'] == data.cpu_name:
                i['temp'] = data.temperature
                i['usage'] = data.cpu_usage
                cpu_exists = True
        if not cpu_exists:
            self.robot.cpulist.append({'name':data.cpu_name, 'temp':data.temperature, 'usage':data.cpu_usage})

    def battery_info_to_object(self, data):
        """Collects information about the current battery state and stores them in self.robot

        Provides current battery voltage and percentage representation of the current battery charge.
        Depends on information from topic /battery_info
        """
        self.robot.battery_voltage = data.voltage
        self.robot.battery_percentage = data.charge_percentage



class Robot:
    """ Stores information about the current robot state, so that information can be published at all times regardless of rates"""
    def __init__(self):
        """Initializes all fields"""
        self.timestamp = 0
        self.motors = [] #list of dictionarys
        self.button1 = False
        self.button2 = False
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.state = -1
        self.cpulist = [] #list of dictionarys
        self.battery_voltage = 0
        self.battery_percentage = 0

    def __str__(self):
        """Humanly readable version of the data currently stored in this classes instance. Can be used for debugging purposes"""
        returnstring = " Timestamp: " + self.timestamp+ "\n \n Motors: \n \n"
        for i in self.motors:
            returnstring = returnstring + str(i) + "\n"
        returnstring = returnstring + "\n Button 1:  " + str(self.button1) + "\n Button 2: " + str(self.button2) + "\n \n" + \
        " Roll: " + str(self.roll) + "\n Pitch: " + str(self.pitch) + "\n Yaw: " + str(self.yaw) + "\n \n State: " + str(self.state) + "\n \n" + \
        " CPU: \n"
        for i in self.cpulist:
            returnstring = returnstring + str(i) + "\n"
        returnstring = returnstring + "\n Battery Voltage: " + str(self.battery_voltage) + "\n Battery Charge: " + str(self.battery_percentage)

        return returnstring





htool = Hardwaretool()
