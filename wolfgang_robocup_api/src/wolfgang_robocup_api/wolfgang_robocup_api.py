#!/usr/bin/env python3

import os
import socket
import yaml
import rospy

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from bitbots_msgs.msg import FootPressure, JointCommand

from wolfgang_robocup_api import messages_pb2


class WolfgangRobocupApi():
    def __init__(self):
        rospack = rospkg.RosPack()
        self._package_path = rospack.get_path("wolfgang_robocup_api")

        rospy.init_node("wolfgang_robocup_api")
        rospy.loginfo("Initializing wolfgang_robocup_api...", logger_name="rc_api")

        self.MIN_FRAME_STEP = rospy.get_param('~min_frame_step')  # ms
        self.MIN_CONTROL_STEP = rospy.get_param('~min_control_step')  # ms

        self.create_publishers()
        self.create_subscribers()

        addr = os.environ.get('ROBOCUP_SIMULATOR_ADDR')
        self.socket = self.get_connection(addr)

    def create_publishers(self):
        self.pub_clock = rospy.Publisher(
            rospy.get_param('~clock_topic'),
            Clock,
            queue_size=1)

    def create_subscribers(self):
        pass

    def get_connection(self, addr):
        rospy.info(f"Connecting to '{addr}'", logger_name="rc_api")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(addr)
        response = sock.recv(8).decode('utf8')
        if response == "Welcome":
            rospy.info(f"Successfully connect to '{addr}'", logger_name="rc_api")
            return sock
        elif response == "Refused":
            rospy.logerr(f"Connection refused by '{addr}'", logger_name="rc_api")
        else:
            rospy.logerr(f"Could not connect to '{addr}'\nGot response '{response}'", logger_name="rc_api")

    def close(self):
        self.socket.close()

    def handle_sensor_measurements_msg(self, msg):
        s_m = messages_pb2.SensorMeasurements()
        s_m.ParseFromString(msg)

        self.handle_time(s_m.time)

    def handle_time(self, time):
        # time stamp at which the measurements were performed expressed in [ms]
        self.time = time
        msg = Clock()
        msg.secs = time // 1000
        msg.nsecs = (time % 1000) * 10**6
        self.pub_clock.publish(msg)

    def handle_real_time(self, time):
        # real unix time stamp at which the measurements were performed in [ms]
        self.real_time = time
