#!/usr/bin/env python3

import os
import socket
import yaml
import rospy
import rospkg
import struct

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from bitbots_msgs.msg import FootPressure, JointCommand

import messages_pb2


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
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            msg_size = self.socket.recv(4)
            msg_size = struct.unpack(">L", msg_size)[0]
            msg = self.socket.recv(msg_size)
            self.handle_sensor_measurements_msg(msg)
        self.close(self.socket)

    def create_publishers(self):
        self.pub_clock = rospy.Publisher(
            rospy.get_param('~clock_topic'),
            Clock,
            queue_size=1)

    def create_subscribers(self):
        pass

    def get_connection(self, addr):
        host, port = addr.split(':')
        port = int(port)
        rospy.loginfo(f"Connecting to '{addr}'", logger_name="rc_api")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        response = sock.recv(8).decode('utf8')
        if response == "Welcome\0":
            rospy.loginfo(f"Successfully connected to '{addr}'", logger_name="rc_api")
            return sock
        elif response == "Refused\0":
            rospy.logerr(f"Connection refused by '{addr}'", logger_name="rc_api")
        else:
            rospy.logerr(f"Could not connect to '{addr}'\nGot response '{response}'", logger_name="rc_api")

    def close(self, sock):
        sock.close()

    def handle_sensor_measurements_msg(self, msg):
        s_m = messages_pb2.SensorMeasurements()
        s_m.ParseFromString(msg)

        self.handle_time(s_m.time)
        self.handle_real_time(s_m.real_time)
        self.handle_messages(s_m.messages)
        self.handle_accelerometer_measurements(s_m.accelerometers)
        self.handle_bumper_measurements(s_m.bumpers)
        self.handle_camera_measurements(s_m.cameras)
        self.handle_force_measurements(s_m.forces)
        self.handle_force3D_measurements(s_m.force3ds)
        self.handle_force6D_measurements(s_m.force6ds)
        self.handle_gyro_measurements(s_m.gyros)
        self.handle_position_sensor_measurements(s_m.position_sensors)

    def handle_time(self, time):
        # time stamp at which the measurements were performed expressed in [ms]
        self.time = time
        msg = Clock()
        msg.clock.secs = time // 1000
        msg.clock.nsecs = (time % 1000) * 10**6
        self.pub_clock.publish(msg)

    def handle_real_time(self, time):
        # real unix time stamp at which the measurements were performed in [ms]
        self.real_time = time

    def handle_messages(self, messages):
        for message in messages:
            text = message.text
            if message.message_type == messages_pb2.Message.ERROR_MESSAGE:
                rospy.logerr(f"RECEIVED ERROR: '{text}'", logger_name="rc_api")
            elif message.message_type == messages_pb2.Message.WARNING_MESSAGE:
                rospy.logwarn(f"RECEIVED WARNING: '{text}'", logger_name="rc_api")
            else:
                rospy.logwarn(f"RECEIVED UNKNOWN MESSAGE: '{text}'", logger_name="rc_api")

    def handle_accelerometer_measurements(self, accelerometers):
        for accelerometer in accelerometers:
            pass

    def handle_bumper_measurements(self, bumpers):
        for bumper in bumpers:
            pass

    def handle_camera_measurements(self, cameras):
        for camera in cameras:
            pass

    def handle_force_measurements(self, forces):
        for force in forces:
            pass

    def handle_force3D_measurements(self, force3ds):
        for force3d in force3ds:
            pass

    def handle_force6D_measurements(self, force6ds):
        for force6d in force6ds:
            pass

    def handle_gyro_measurements(self, gyros):
        for gyro in gyros:
            pass

    def handle_position_sensor_measurements(self, position_sensors):
        for position_sensor in position_sensors:
            pass


if __name__ == '__main__':
    WolfgangRobocupApi()
