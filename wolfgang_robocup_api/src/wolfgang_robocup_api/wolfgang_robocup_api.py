#!/usr/bin/env python3

import os
import sys
import math
import re
import socket
import time

import rospy
import rospkg
import struct
import copy
from threading import Lock
from urdf_parser_py.urdf import URDF

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
        self.camera_FOV = rospy.get_param('~camera_FOV')

        self.camera_optical_frame = rospy.get_param('~camera_optical_frame')
        self.imu_frame = rospy.get_param('~imu_frame')
        self.head_imu_frame = rospy.get_param('~head_imu_frame')
        self.l_sole_frame = rospy.get_param('~l_sole_frame')
        self.r_sole_frame = rospy.get_param('~r_sole_frame')

        # Parse URDF
        urdf_path = os.path.join(rospack.get_path('wolfgang_description'), 'urdf', 'robot.urdf')
        urdf = URDF.from_xml_file(urdf_path)
        joints = [joint for joint in urdf.joints if joint.type == 'revolute']
        self.velocity_limits = {joint.name: joint.limit.velocity for joint in joints}
        joint_names = [joint.name for joint in joints]

        self.position_sensors = [self.joint_to_webots(name) + "_sensor" for name in joint_names]
        self.force3d_sensors = [
            "llb",
            "llf",
            "lrf",
            "lrb",
            "rlb",
            "rlf",
            "rrf",
            "rrb",
        ]
        self.sensors_names = [
            "camera",
            "imu accelerometer",
            "imu gyro",
            "imu_head accelerometer",
            "imu_head gyro",
            ]
        self.sensors_names.extend(self.position_sensors)
        self.sensors_names.extend(self.force3d_sensors)

        self.positions = {}
        self.velocities = {}
        self.torques = {}

        self.create_publishers()
        self.create_subscribers()

        addr = os.environ.get('ROBOCUP_SIMULATOR_ADDR')
        # we will try multiple times till we manage to get a connection
        self.socket = None
        while not rospy.is_shutdown() and self.socket is None:
            self.socket = self.get_connection(addr)
            time.sleep(1) #dont use ros time since it is maybe not available

        self.first_run = True
        self.published_camera_info = False

        self.joint_command_mutex = Lock()

        self.run()

    def receive_msg(self):
        msg_size = self.socket.recv(4)
        msg_size = struct.unpack(">L", msg_size)[0]

        data = bytearray()
        while len(data) < msg_size:
            packet = self.socket.recv(msg_size - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data

    def run(self):
        while not rospy.is_shutdown():
            # Parse sensor
            msg = self.receive_msg()
            if msg:  # Not handle empty messages or None
                self.handle_sensor_measurements_msg(msg)

            sensor_time_steps = None
            if self.first_run:
                sensor_time_steps = self.get_sensor_time_steps(active=True)
            self.send_actuator_requests(sensor_time_steps)
            self.first_run = False
        self.close_connection()

    def create_publishers(self):
        self.pub_clock = rospy.Publisher(rospy.get_param('~clock_topic'), Clock, queue_size=1)
        self.pub_server_time_clock = rospy.Publisher(rospy.get_param('~server_time_clock_topic'), Clock, queue_size=1)
        self.pub_camera = rospy.Publisher(rospy.get_param('~camera_topic'), Image, queue_size=1)
        self.pub_camera_info = rospy.Publisher(rospy.get_param('~camera_info_topic'), CameraInfo, queue_size=1, latch=True)
        self.pub_imu = rospy.Publisher(rospy.get_param('~imu_topic'), Imu, queue_size=1)
        self.pub_head_imu = rospy.Publisher(rospy.get_param('~imu_head_topic'), Imu, queue_size=1)
        self.pub_pressure_left = rospy.Publisher(rospy.get_param('~foot_pressure_left_topic'), FootPressure, queue_size=1)
        self.pub_pressure_right = rospy.Publisher(rospy.get_param('~foot_pressure_right_topic'), FootPressure, queue_size=1)
        self.pub_cop_l = rospy.Publisher(rospy.get_param('~cop_left_topic'), PointStamped, queue_size=1)
        self.pub_cop_r_ = rospy.Publisher(rospy.get_param('~cop_right_topic'), PointStamped, queue_size=1)
        self.pub_joint_states = rospy.Publisher(rospy.get_param('~joint_states_topic'), JointState, queue_size=1)

    def create_subscribers(self):
        self.sub_joint_command = rospy.Subscriber(rospy.get_param('~joint_command_topic'), JointCommand, self.joint_command_cb, queue_size=1)

    def joint_command_cb(self, msg):
        with self.joint_command_mutex:
            for i, name in enumerate(msg.joint_names):
                webots_name = self.joint_to_webots(name)
                if len(msg.positions) == 0:
                    # no positions given, use torque control
                    self.torques[webots_name] = msg.accelerations[i]
                    # delete positon and velocities
                    if webots_name in self.positions:
                        del self.positions[webots_name]
                    if webots_name in self.velocities:
                        del self.velocities[webots_name]
                else:
                    # use position and velocity control
                    self.positions[webots_name] = msg.positions[i]

                    if len(msg.velocities) == 0 or msg.velocities[i] == -1:
                        self.velocities[webots_name] = self.velocity_limits[name]
                    else:
                        self.velocities[webots_name] = msg.velocities[i]

                    if webots_name in self.torques:
                        del self.torques[webots_name]

    def get_connection(self, addr):
        host, port = addr.split(':')
        port = int(port)
        rospy.loginfo(f"Connecting to '{addr}'", logger_name="rc_api")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect((host, port))
            response = sock.recv(8).decode('utf8')
        except ConnectionRefusedError:
            rospy.logerr(f"Connection refused by '{addr}'", logger_name="rc_api")
            return None
        if response == "Welcome\0":
            rospy.loginfo(f"Successfully connected to '{addr}'", logger_name="rc_api")
            return sock
        elif response == "Refused\0":
            rospy.logerr(f"Connection refused by '{addr}'", logger_name="rc_api")
            return None
        else:
            rospy.logerr(f"Could not connect to '{addr}'\nGot response '{response}'", logger_name="rc_api")
            return None

    def close_connection(self):
        if self.socket:
            self.socket.close()
            rospy.loginfo("Connection closed.", logger_name="rc_api")

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
        msg.clock.nsecs = ros_time.nsecs
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
        imu_msg.orientation.w = 1
        imu_accel = imu_gyro = False

        # Head IMU
        head_imu_msg = Imu()
        head_imu_msg.header.stamp = self.stamp
        head_imu_msg.header.frame_id = self.head_imu_frame
        head_imu_msg.orientation.w = 1
        head_imu_accel = head_imu_gyro = False

        # Extract data from message
        for accelerometer in accelerometers:
            name = accelerometer.name
            value = accelerometer.value
            if name == "imu accelerometer":
                imu_accel = True
                imu_msg.linear_acceleration.x = value.X
                imu_msg.linear_acceleration.y = value.Y
                imu_msg.linear_acceleration.z = value.Z
            elif name == "imu_head accelerometer":
                head_imu_accel = True
                head_imu_msg.linear_acceleration.x = value.Z
                head_imu_msg.linear_acceleration.y = value.X
                head_imu_msg.linear_acceleration.z = value.Y
            else:
                rospy.logwarn(f"Unknown accelerometer: '{name}'", logger_name="rc_api")

        for gyro in gyros:
            name = gyro.name
            value = gyro.value
            if name == "imu gyro":
                imu_gyro = True
                imu_msg.angular_velocity.x = value.X
                imu_msg.angular_velocity.y = value.Y
                imu_msg.angular_velocity.z = value.Z
            elif name == "imu_head gyro":
                head_imu_gyro = True
                head_imu_msg.angular_velocity.x = value.Z
                head_imu_msg.angular_velocity.y = value.X
                head_imu_msg.angular_velocity.z = value.Y
            else:
                rospy.logwarn(f"Unknown gyro: '{name}'", logger_name="rc_api")

        if imu_accel and imu_gyro:
            # Make sure that acceleration is not completely zero or we will get error in filter.
            # Happens if robot is moved manually in the simulation
            if imu_msg.linear_acceleration.x == 0 and imu_msg.linear_acceleration.y == 0 and imu_msg.linear_acceleration.z == 0:
                imu_msg.linear_acceleration.z = 0.001

            self.pub_imu.publish(imu_msg)
        if head_imu_accel and head_imu_gyro:
            # Same as above
            if head_imu_msg.linear_acceleration.x == 0 and head_imu_msg.linear_acceleration.y == 0 and head_imu_msg.linear_acceleration.z == 0:
                head_imu_msg.linear_acceleration.z = 0.001

            self.pub_head_imu.publish(head_imu_msg)

    def handle_bumper_measurements(self, bumpers):
        for bumper in bumpers:
            rospy.logwarn(f"Unknown bumper: '{bumper.name}'", logger_name="rc_api")

    def handle_camera_measurements(self, cameras):
        for camera in cameras:
            name = camera.name
            if name == "camera":
                width = camera.width
                height = camera.height
                quality = camera.quality  # 1 = raw image, 100 = no compression, 0 = high compression
                image = camera.image  # RAW or JPEG encoded data (note: JPEG is not yet implemented)

                if not self.published_camera_info:  # Publish CameraInfo once, it will be latched
                    self.publish_camera_info(height, width)
                    self.published_camera_info = True

                img_msg = Image()
                img_msg.header.stamp = self.stamp
                img_msg.header.frame_id = self.camera_optical_frame
                img_msg.height = height
                img_msg.width = width
                img_msg.encoding = "bgr8"
                img_msg.step = 3 * width
                img_msg.data = image
                self.pub_camera.publish(img_msg)
            else:
                rospy.logwarn(f"Unknown camera: '{name}'", logger_name="rc_api")

    def publish_camera_info(self, height, width):
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = self.stamp
        camera_info_msg.header.frame_id = self.camera_optical_frame
        camera_info_msg.height = height
        camera_info_msg.width = width
        f_y = self.mat_from_fov_and_resolution(
            self.h_fov_to_v_fov(self.camera_FOV, height, width),
            height)
        f_x = self.mat_from_fov_and_resolution(self.camera_FOV, width)
        camera_info_msg.K = [f_x, 0, width / 2,
                        0, f_y, height / 2,
                        0, 0, 1]
        camera_info_msg.P = [f_x, 0, width / 2, 0,
                        0, f_y, height / 2, 0,
                        0, 0, 1, 0]
        self.pub_camera_info.publish(camera_info_msg)

    def mat_from_fov_and_resolution(self, fov, res):
        return 0.5 * res * (math.cos((fov / 2)) / math.sin((fov / 2)))

    def h_fov_to_v_fov(self, h_fov, height, width):
        return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

    def handle_force_measurements(self, forces):
        if not forces:
            return

        data = {}
        for force in forces:
            name = force.name
            if name in self.force3d_sensors:
                data[name] = force.value
            else:
                rospy.logwarn(f"Unknown force measurement: '{name}'", logger_name="rc_api")

        left_pressure_msg = FootPressure()
        left_pressure_msg.header.stamp = self.stamp
        # TODO: Frame IDs
        left_pressure_msg.left_back = data['llb']
        left_pressure_msg.left_front = data['llf']
        left_pressure_msg.right_front = data['lrf']
        left_pressure_msg.right_back = data['lrb']

        right_pressure_msg = FootPressure()
        right_pressure_msg.header.stamp = self.stamp
        # TODO: Frame IDs
        right_pressure_msg.left_back = data['rlb']
        right_pressure_msg.left_front = data['rlf']
        right_pressure_msg.right_front = data['rrf']
        right_pressure_msg.right_back = data['rrb']

        # compute center of pressures of the feet
        pos_x = 0.085
        pos_y = 0.045
        # we can take a very small threshold, since simulation gives more accurate values than reality
        threshold = 1

        cop_l_msg = PointStamped()
        cop_l_msg.header.stamp = self.stamp
        cop_l_msg.header.frame_id = self.l_sole_frame
        sum = left_pressure_msg.left_back + left_pressure_msg.left_front + left_pressure_msg.right_front + left_pressure_msg.right_back
        if sum > threshold:
            cop_l_msg.point.x = (left_pressure_msg.left_front + left_pressure_msg.right_front -
                             left_pressure_msg.left_back - left_pressure_msg.right_back) * pos_x / sum
            cop_l_msg.point.x = max(min(cop_l_msg.point.x, pos_x), -pos_x)
            cop_l_msg.point.y = (left_pressure_msg.left_front + left_pressure_msg.left_back -
                             left_pressure_msg.right_front - left_pressure_msg.right_back) * pos_y / sum
            cop_l_msg.point.y = max(min(cop_l_msg.point.x, pos_y), -pos_y)
        else:
            cop_l_msg.point.x = 0
            cop_l_msg.point.y = 0

        cop_r_msg = PointStamped()
        cop_r_msg.header.stamp = self.stamp
        cop_r_msg.header.frame_id = self.r_sole_frame
        sum = right_pressure_msg.right_back + right_pressure_msg.right_front + right_pressure_msg.right_front + right_pressure_msg.right_back
        if sum > threshold:
            cop_r_msg.point.x = (right_pressure_msg.left_front + right_pressure_msg.right_front -
                             right_pressure_msg.left_back - right_pressure_msg.right_back) * pos_x / sum
            cop_r_msg.point.x = max(min(cop_r_msg.point.x, pos_x), -pos_x)
            cop_r_msg.point.y = (right_pressure_msg.left_front + right_pressure_msg.left_back -
                             right_pressure_msg.right_front - right_pressure_msg.right_back) * pos_y / sum
            cop_r_msg.point.y = max(min(cop_r_msg.point.x, pos_y), -pos_y)
        else:
            cop_r_msg.point.x = 0
            cop_r_msg.point.y = 0

        self.pub_pressure_left.publish(left_pressure_msg)
        self.pub_pressure_right.publish(right_pressure_msg)
        self.pub_cop_l.publish(cop_l_msg)
        self.pub_cop_r_.publish(cop_r_msg)

    def handle_force3D_measurements(self, force3ds):
        for force3d in force3ds:
            rospy.logwarn(f"Unknown force3d measurement: '{force3d.name}'", logger_name="rc_api")

    def handle_force6D_measurements(self, force6ds):
        for force6d in force6ds:
            rospy.logwarn(f"Unknown force6d measurement: '{force6d.name}'", logger_name="rc_api")

    def handle_position_sensor_measurements(self, position_sensors):
        state_msg = JointState()
        state_msg.header.stamp = self.stamp
        for position_sensor in position_sensors:
            state_msg.name.append(self.webots_to_joint(position_sensor.name))
            state_msg.position.append(position_sensor.value)
        self.pub_joint_states.publish(state_msg)

    def get_sensor_time_steps(self, active=True):
        sensor_time_steps = []
        for sensor_name in self.sensors_names:
            time_step = self.MIN_CONTROL_STEP
            if sensor_name == "camera":
                time_step = self.MIN_FRAME_STEP
            if not active:
                time_step = 0
            sensor_time_step = messages_pb2.SensorTimeStep()
            sensor_time_step.name = sensor_name
            sensor_time_step.timeStep = time_step
            sensor_time_steps.append(sensor_time_step)
        return sensor_time_steps

    def send_actuator_requests(self, sensor_time_steps=None):
        actuator_requests = messages_pb2.ActuatorRequests()
        if sensor_time_steps is not None:
            actuator_requests.sensor_time_steps.extend(sensor_time_steps)

        # Makes this this thread safe
        with self.joint_command_mutex:
            for name in self.positions:
                motor_position = messages_pb2.MotorPosition()
                motor_position.name = name
                motor_position.position = self.positions[name]
                actuator_requests.motor_positions.append(motor_position)

                motor_velocity = messages_pb2.MotorVelocity()
                motor_velocity.name = name
                motor_velocity.velocity = self.velocities[name]
                actuator_requests.motor_velocities.append(motor_velocity)

            for name in self.torques:
                motor_torque = messages_pb2.MotorTorque()
                motor_torque.name = name
                motor_torque.torque = self.torques[name]
                actuator_requests.motor_torques.append(motor_torque)

        msg = actuator_requests.SerializeToString()
        msg_size = struct.pack(">L", len(msg))
        self.socket.send(msg_size + msg)

    def joint_to_webots(self, joint):
        if joint in ('RHipRoll', 'LHipRoll'):
            return joint + ' [hip]'
        elif joint in ('RShoulderPitch', 'LShoulderPitch'):
            return joint + ' [shoulder]'
        else:
            return joint

    def webots_to_joint(self, name):
        return re.sub(r'( \[\w+\])?_sensor', '', name)

if __name__ == '__main__':
    WolfgangRobocupApi()
