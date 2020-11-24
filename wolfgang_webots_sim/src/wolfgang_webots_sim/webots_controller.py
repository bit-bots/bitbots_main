import subprocess
import time

import tf
import os
try:
    from controller import Robot, Node, Supervisor, Field
except:
    env_file = os.path.realpath(os.path.join(os.path.dirname(__file__), '../../scripts/setenvs.sh'))
    exit(f'Please execute "source {env_file}" first')
import rospy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState, Imu, Image
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty

from bitbots_msgs.msg import JointCommand
import math
from tf.transformations import quaternion_from_euler

G = 9.81


class WebotsController:
    def __init__(self, namespace='', ros_active=False, mode='normal', robot='wolfgang'):
        self.ros_active = ros_active
        self.time = 0
        self.clock_msg = Clock()
        self.namespace = namespace
        self.supervisor = Supervisor()

        self.walkready = [0] * 20

        self.motors = []
        self.sensors = []
        self.timestep = int(self.supervisor.getBasicTimeStep())

        if mode == 'normal':
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
        elif mode == 'paused':
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
        elif mode == 'run':
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_RUN)
        elif mode == 'fast':
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)
        else:
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)

        self.robot_name = robot
        self.switch_coordinate_system = True
        self.pressure_sensors = None
        if robot == 'wolfgang':
            self.robot_node_name = "Robot"
            self.motor_names = ["RShoulderPitch", "LShoulderPitch", "RShoulderRoll", "LShoulderRoll", "RElbow",
                                "LElbow", "RHipYaw", "LHipYaw", "RHipRoll", "LHipRoll", "RHipPitch", "LHipPitch",
                                "RKnee", "LKnee", "RAnklePitch", "LAnklePitch", "RAnkleRoll", "LAnkleRoll", "HeadPan",
                                "HeadTilt"]
            self.external_motor_names = self.motor_names
            sensor_postfix = "_sensor"
            accel_name = "imu accelerometer"
            gyro_name = "imu gyro"
            camera_name = "camera"
            pressure_sensor_names = ["LLB", "LLF", "LRF", "LRB", "RLB", "RLF", "RRF", "RRB"]
            self.pressure_sensors = []
            for name in pressure_sensor_names:
                self.accel = self.supervisor.getTouch(name)
            print(self.pressure_sensors)
        elif robot == 'darwin':
            self.robot_node_name = "Darwin"
            self.motor_names = ["ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR", "ArmLowerL",
                                "PelvYR", "PelvYL", "PelvR", "PelvL", "LegUpperR", "LegUpperL", "LegLowerR",
                                "LegLowerL", "AnkleR", "AnkleL", "FootR", "FootL", "Neck", "Head"]
            self.external_motor_names = ["RShoulderPitch", "LShoulderPitch", "RShoulderRoll", "LShoulderRoll", "RElbow",
                                         "LElbow", "RHipYaw", "LHipYaw", "RHipRoll", "LHipRoll", "RHipPitch",
                                         "LHipPitch", "RKnee", "LKnee", "RAnklePitch", "LAnklePitch", "RAnkleRoll",
                                         "LAnkleRoll", "HeadPan", "HeadTilt"]
            sensor_postfix = "S"
            accel_name = "Accelerometer"
            gyro_name = "Gyro"
            camera_name = "Camera"
        elif robot == 'nao':
            self.robot_node_name = "Robot"
            self.motor_names = ["RShoulderPitch", "LShoulderPitch", "RShoulderRoll", "LShoulderRoll", "RElbowYaw",
                                "LElbowYaw", "RHipYawPitch", "LHipYawPitch", "RHipRoll", "LHipRoll", "RHipPitch",
                                "LHipPitch",
                                "RKneePitch", "LKneePitch", "RAnklePitch", "LAnklePitch", "RAnkleRoll", "LAnkleRoll",
                                "HeadYaw",
                                "HeadPitch"]
            self.external_motor_names = self.motor_names
            sensor_postfix = "S"
            accel_name = "accelerometer"
            gyro_name = "gyro"
            camera_name = "CameraTop"
            self.switch_coordinate_system = False
        elif robot == 'op3':
            self.robot_node_name = "Robot"
            self.motor_names = ["ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR", "ArmLowerL",
                                "PelvYR", "PelvYL", "PelvR", "PelvL", "LegUpperR", "LegUpperL", "LegLowerR",
                                "LegLowerL", "AnkleR", "AnkleL", "FootR", "FootL", "Neck", "Head"]
            self.external_motor_names = ["r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll",
                                         "r_el", "l_el", "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll",
                                         "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee", "r_ank_pitch",
                                         "l_ank_pitch", "r_ank_roll", "l_ank_roll", "head_pan", "head_tilt"]
            sensor_postfix = "S"
            accel_name = "Accelerometer"
            gyro_name = "Gyro"
            camera_name = "Camera"
            self.switch_coordinate_system = False

        self.robot_node = self.supervisor.getFromDef(self.robot_node_name)
        for motor_name in self.motor_names:
            self.motors.append(self.supervisor.getMotor(motor_name))
            self.motors[-1].enableTorqueFeedback(self.timestep)
            self.sensors.append(self.supervisor.getPositionSensor(motor_name + sensor_postfix))
            self.sensors[-1].enable(self.timestep)

        self.accel = self.supervisor.getAccelerometer(accel_name)
        self.accel.enable(self.timestep)
        self.gyro = self.supervisor.getGyro(gyro_name)
        self.gyro.enable(self.timestep)
        self.camera = self.supervisor.getCamera(camera_name)
        self.camera.enable(self.timestep)

        if self.ros_active:
            rospy.init_node("webots_ros_interface", anonymous=True,
                            argv=['clock:=/' + self.namespace + '/clock'])
        self.pub_js = rospy.Publisher(self.namespace + "/joint_states", JointState, queue_size=1)
        self.pub_imu = rospy.Publisher(self.namespace + "/imu/data", Imu, queue_size=1)
        self.pub_cam = rospy.Publisher(self.namespace + "/image_raw", Image, queue_size=1)
        self.clock_publisher = rospy.Publisher(self.namespace + "/clock", Clock, queue_size=1)
        rospy.Subscriber(self.namespace + "/DynamixelController/command", JointCommand, self.command_cb)

        self.translation_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")
        self.world_info = self.supervisor.getFromDef("world_info")

        self.reset_service = rospy.Service("reset", Empty, self.reset)

    def step_sim(self):
        self.time += self.timestep / 1000
        self.supervisor.step(self.timestep)

    def step(self):
        self.step_sim()
        if self.ros_active:
            self.publish_imu()
            self.publish_joint_states()
            self.publish_camera()
            self.publish_clock()

    def publish_clock(self):
        self.clock_msg.clock = rospy.Time.from_seconds(self.time)
        self.clock_publisher.publish(self.clock_msg)

    def command_cb(self, command: JointCommand):
        for i, name in enumerate(command.joint_names):
            try:
                motor_index = self.external_motor_names.index(name)
                self.motors[motor_index].setPosition(command.positions[i])
            except ValueError:
                print(f"invalid motor specified ({name})")

    def set_head_tilt(self, pos):
        self.motors[-1].setPosition(pos)

    def set_arms_zero(self):
        positions = [-0.8399999308200574, 0.7200000596634105, -0.3299999109923385, 0.35999992683575216,
                     0.5099999812500172, -0.5199999789619728]
        for i in range(0, 6):
            self.motors[i].setPosition(positions[i])

    def get_joint_state_msg(self):
        js = JointState()
        js.name = []
        js.header.stamp = rospy.Time.from_seconds(self.time)
        js.position = []
        js.effort = []
        for i in range(len(self.sensors)):
            js.name.append(self.external_motor_names[i])
            value = self.sensors[i].getValue()
            js.position.append(value)
            js.effort.append(self.motors[i].getTorqueFeedback())
        if self.ros_active:
            self.pub_js.publish(js)
        return js

    def publish_joint_states(self):
        self.pub_js.publish(self.get_joint_state_msg())

    def get_imu_msg(self):
        msg = Imu()
        msg.header.stamp = rospy.Time.from_seconds(self.time)
        msg.header.frame_id = "imu_frame"

        # change order because webots has different axis
        accel_vels = self.accel.getValues()
        msg.linear_acceleration.x = accel_vels[2]
        msg.linear_acceleration.y = -accel_vels[0]
        msg.linear_acceleration.z = accel_vels[1]

        gyro_vels = self.gyro.getValues()
        msg.angular_velocity.x = gyro_vels[2]
        msg.angular_velocity.y = -gyro_vels[0]
        msg.angular_velocity.z = gyro_vels[1]

        pos, rpy = self.get_robot_pose_rpy()
        quat_tf = quaternion_from_euler(*rpy)
        msg.orientation = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
        return msg

    def publish_imu(self):
        self.pub_imu.publish(self.get_imu_msg())

    def publish_camera(self):
        img_msg = Image()
        img_msg.header.stamp = rospy.Time.from_seconds(self.time)
        img_msg.height = self.camera.getHeight()
        img_msg.width = self.camera.getWidth()
        img_msg.encoding = "bgra8"
        img_msg.step = 4 * self.camera.getWidth()
        img = self.camera.getImage()
        img_msg.data = img
        self.pub_cam.publish(img_msg)

    def get_image(self):
        return self.camera.getImage()

    def set_gravity(self, active):
        if active:
            self.world_info.getField("gravity").setSFVec3f([0.0, -9.81, 0.0])
            self.world_info.getField("gravity").setSFFloat(9.81)
        else:
            self.world_info.getField("gravity").setSFVec3f([0.0, 0.0, 0.0])
            self.world_info.getField("gravity").setSFFloat(0)

    def reset_robot_pose(self, pos, quat):
        rpy = tf.transformations.euler_from_quaternion(quat)
        self.set_robot_pose_rpy(pos, rpy)
        self.robot_node.resetPhysics()

    def reset_robot_pose_rpy(self, pos, rpy):
        self.set_robot_pose_rpy(pos, rpy)
        self.robot_node.resetPhysics()

    def reset(self, req=None):
        self.supervisor.simulationReset()
        self.supervisor.simulationResetPhysics()

    def node(self):
        s = self.supervisor.getSelected()
        if s is not None:
            print(f"id: {s.getId()}, type: {s.getType()}, def: {s.getDef()}")

    def set_robot_pose_rpy(self, pos, rpy):
        if self.switch_coordinate_system:
            self.translation_field.setSFVec3f(pos_ros_to_webots(pos))
            self.rotation_field.setSFRotation(rpy_to_axis(*rpy))
        else:
            self.translation_field.setSFVec3f([pos[0], pos[1], pos[2]])
            self.rotation_field.setSFRotation(rpy_to_axis(rpy[1], rpy[2], rpy[0]))

    def set_robot_rpy(self, rpy):
        if self.switch_coordinate_system:
            self.rotation_field.setSFRotation(rpy_to_axis(*rpy))
        else:
            self.rotation_field.setSFRotation(rpy_to_axis(rpy[1], rpy[2], rpy[0]))

    def get_robot_pose_rpy(self):
        pos = self.translation_field.getSFVec3f()
        rot = self.rotation_field.getSFRotation()
        if self.switch_coordinate_system:
            return pos_webots_to_ros(pos), axis_to_rpy(*rot)
        else:
            rpy = axis_to_rpy(*rot)
            return pos, (rpy[2], rpy[0], rpy[1])


def pos_webots_to_ros(pos):
    x = pos[2]
    y = pos[0]
    z = pos[1]
    return [x, y, z]


def pos_ros_to_webots(pos):
    z = pos[0]
    x = pos[1]
    y = pos[2]
    return [x, y, z]


def rpy_to_axis(z_e, x_e, y_e, normalize=True):
    # Assuming the angles are in radians.
    c1 = math.cos(z_e / 2)
    s1 = math.sin(z_e / 2)
    c2 = math.cos(x_e / 2)
    s2 = math.sin(x_e / 2)
    c3 = math.cos(y_e / 2)
    s3 = math.sin(y_e / 2)
    c1c2 = c1 * c2
    s1s2 = s1 * s2
    w = c1c2 * c3 - s1s2 * s3
    x = c1c2 * s3 + s1s2 * c3
    y = s1 * c2 * c3 + c1 * s2 * s3
    z = c1 * s2 * c3 - s1 * c2 * s3
    angle = 2 * math.acos(w)
    if normalize:
        norm = x * x + y * y + z * z
        if norm < 0.001:
            # when all euler angles are zero angle =0 so
            # we can set axis to anything to avoid divide by zero
            x = 1
            y = 0
            z = 0
        else:
            norm = math.sqrt(norm)
            x /= norm
            y /= norm
            z /= norm
    return [z, x, y, angle]


def axis_to_rpy(x, y, z, angle):
    s = math.sin(angle)
    c = math.cos(angle)
    t = 1 - c

    magnitude = math.sqrt(x * x + y * y + z * z)
    if magnitude == 0:
        raise AssertionError
    x /= magnitude
    y /= magnitude
    z /= magnitude
    # north pole singularity
    if (x * y * t + z * s) > 0.998:
        yaw = 2 * math.atan2(x * math.sin(angle / 2), math.cos(angle / 2))
        pitch = math.pi / 2
        roll = 0
        return roll, pitch, yaw

    # south pole singularity
    if (x * y * t + z * s) < -0.998:
        yaw = -2 * math.atan2(x * math.sin(angle / 2), math.cos(angle / 2))
        pitch = -math.pi / 2
        roll = 0
        return roll, pitch, yaw

    yaw = math.atan2(y * s - x * z * t, 1 - (y * y + z * z) * t)
    pitch = math.asin(x * y * t + z * s)
    roll = math.atan2(x * s - y * z * t, 1 - (x * x + z * z) * t)

    return roll, pitch, yaw
