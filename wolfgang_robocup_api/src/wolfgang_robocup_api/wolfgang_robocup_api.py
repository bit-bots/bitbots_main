#!/usr/bin/env python3

import os
import socket
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

        self.imu_frame = rospy.get_param('~imu_frame')
        self.head_imu_frame = rospy.get_param('~head_imu_frame')

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
            self.send_actuator_requests(self.socket)
        self.close(self.socket)

    def create_publishers(self):
        self.pub_clock = rospy.Publisher(rospy.get_param('~clock_topic'), Clock, queue_size=1)
        self.pub_server_time_clock = rospy.Publisher(rospy.get_param('~server_time_clock_topic'), Clock, queue_size=1)
        self.pub_imu = rospy.Publisher(rospy.get_param('~imu_topic'), Imu, queue_size=1)
        self.pub_head_imu = rospy.Publisher(rospy.get_param('~imu_head_topic'), Imu, queue_size=1)

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
        self.handle_imu_data(s_m.accelerometers, s_m.gyros)
        self.handle_bumper_measurements(s_m.bumpers)
        self.handle_camera_measurements(s_m.cameras)
        self.handle_force_measurements(s_m.forces)
        self.handle_force3D_measurements(s_m.force3ds)
        self.handle_force6D_measurements(s_m.force6ds)
        self.handle_position_sensor_measurements(s_m.position_sensors)

    def handle_time(self, time):
        # time stamp at which the measurements were performed expressed in [ms]
        secs = time / 1000
        ros_time = rospy.Time.from_seconds(secs)
        self.stamp = ros_time
        msg = Clock()
        msg.clock.secs = ros_time.secs
        msg.clock.nsecs = ros_time.nsec
        self.pub_clock.publish(msg)

    def handle_real_time(self, time):
        # real unix time stamp at which the measurements were performed in [ms]
        msg = Clock()
        msg.clock.secs = time // 1000
        msg.clock.nsecs = (time % 1000) * 10**6
        self.pub_server_time_clock.publish(msg)

    def handle_messages(self, messages):
        for message in messages:
            text = message.text
            if message.message_type == messages_pb2.Message.ERROR_MESSAGE:
                rospy.logerr(f"RECEIVED ERROR: '{text}'", logger_name="rc_api")
            elif message.message_type == messages_pb2.Message.WARNING_MESSAGE:
                rospy.logwarn(f"RECEIVED WARNING: '{text}'", logger_name="rc_api")
            else:
                rospy.logwarn(f"RECEIVED UNKNOWN MESSAGE: '{text}'", logger_name="rc_api")

    def handle_imu_data(self, accelerometers, gyros):
        # Body IMU
        imu_msg = Imu()
        imu_msg.header.stamp = self.stamp
        imu_msg.header.frame_id = self.imu_frame
        imu_accel = imu_gyro = False

        # Head IMU
        head_imu_msg = Imu()
        head_imu_msg.header.stamp = self.stamp
        head_imu_msg.header.frame_id = self.head_imu_frame
        head_imu_accel = head_imu_gyro = False

        # Extract data from message
        for accelerometer in accelerometers:
            name = accelerometer.name
            value = accelerometer.value
            if name == "imu accelerometer":
                imu_accel = True
                imu_msg.linear_acceleration.x = value[0]
                imu_msg.linear_acceleration.y = value[1]
                imu_msg.linear_acceleration.z = value[2]
            elif name == "imu_head accelerometer":
                head_imu_accel = True
                head_imu_msg.linear_acceleration.x = value[2]
                head_imu_msg.linear_acceleration.y = value[0]
                head_imu_msg.linear_acceleration.z = value[1]
            else:
                rospy.logwarn(f"Unknown accelerometer: '{name}'", logger_name="rc_api")

        for gyro in gyros:
            name = gyros.name
            value = gyros.value
            if name == "imu gyro":
                imu_gyro = True
                imu_msg.linear_acceleration.x = value[0]
                imu_msg.linear_acceleration.y = value[1]
                imu_msg.linear_acceleration.z = value[2]
            elif name == "imu_head gyro":
                head_imu_gyro = True
                head_imu_msg.linear_acceleration.x = value[2]
                head_imu_msg.linear_acceleration.y = value[0]
                head_imu_msg.linear_acceleration.z = value[1]
            else:
                rospy.logwarn(f"Unknown gyro: '{name}'", logger_name="rc_api")

        if imu_accel and imu_gyro:
            self.pub_imu(imu_msg)
        if head_imu_accel and head_imu_gyro:
            self.pub_head_imu(head_imu_msg)

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

    def send_actuator_requests(self, sock):
        sensor_time_step = messages_pb2.SensorTimeStep()
        sensor_time_step.name = "imu accelerometer"
        sensor_time_step.timeStep = self.MIN_CONTROL_STEP
        motor_velocity = messages_pb2.MotorVelocity()
        motor_velocity.name = "HeadTilt"
        motor_velocity.velocity = 8.17
        motor_position = messages_pb2.MotorPosition()
        motor_position.name = "HeadTilt"
        motor_position.position = -1.0

        a_r = messages_pb2.ActuatorRequests()
        #a_r.sensor_time_steps.append(sensor_time_step)
        a_r.motor_positions.append(motor_position)
        a_r.motor_velocities.append(motor_velocity)
        msg = a_r.SerializeToString()
        msg_size = struct.pack(">L", len(msg))
        sock.send(msg_size + msg)


if __name__ == '__main__':
    WolfgangRobocupApi()
