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
        print(s_m)

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

        # self.publish_imus()

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

    def handle_time(self, time):
        # time stamp at which the measurements were performed expressed in [ms]
        self.time = time
        msg = Clock()
        msg.clock.secs = time // 1000
        msg.clock.nsecs = (time % 1000) * 10**6
        self.pub_clock.publish(msg)

    def handle_real_time(self, time):
        # real unix time stamp at which the measurements were performed in [ms]
        self.server_time = time
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

    def get_imu_msg(self, head=False):
        msg = Imu()
        msg.header.stamp = rospy.Time.from_seconds(self.time)
        if head:
            msg.header.frame_id = self.head_imu_frame
        else:
            msg.header.frame_id = self.imu_frame

        # change order because webots has different axis
        if head:
            accel_vels = self.accel_head.getValues()
            msg.linear_acceleration.x = accel_vels[2]
            msg.linear_acceleration.y = -accel_vels[0]
            msg.linear_acceleration.z = -accel_vels[1]
        else:
            accel_vels = self.accel.getValues()
            msg.linear_acceleration.x = accel_vels[0]
            msg.linear_acceleration.y = accel_vels[1]
            msg.linear_acceleration.z = accel_vels[2]

        if head:
            gyro_vels = self.gyro_head.getValues()
            msg.angular_velocity.x = gyro_vels[2]
            msg.angular_velocity.y = -gyro_vels[0]
            msg.angular_velocity.z = -gyro_vels[1]
        else:
            gyro_vels = self.gyro.getValues()
            msg.angular_velocity.x = gyro_vels[0]
            msg.angular_velocity.y = gyro_vels[1]
            msg.angular_velocity.z = gyro_vels[2]
        return msg

    def publish_imus(self):
        self.pub_imu(self.get_imu_msg(head=False))
        self.pub_head_imu(self.get_imu_msg(head=True))


if __name__ == '__main__':
    WolfgangRobocupApi()
