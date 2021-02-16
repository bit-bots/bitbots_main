import subprocess
import time

import os

from controller import Robot, Node, Supervisor, Field

import rospy
from geometry_msgs.msg import Quaternion, PointStamped
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo

from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty

from bitbots_msgs.msg import JointCommand, FootPressure
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import transforms3d
import numpy as np

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
        self.is_wolfgang = False
        self.pressure_sensors = None
        if robot == 'wolfgang':
            self.is_wolfgang = True
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
            pressure_sensor_names = ["llb", "llf", "lrf", "lrb", "rlb", "rlf", "rrf", "rrb"]
            self.pressure_sensors = []
            for name in pressure_sensor_names:
                sensor = self.supervisor.getTouchSensor(name)
                sensor.enable(30)
                self.pressure_sensors.append(sensor)

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
        if self.is_wolfgang:
            self.accel_head = self.supervisor.getAccelerometer("imu_head accelerometer")
            self.accel_head.enable(self.timestep)
            self.gyro_head = self.supervisor.getGyro("imu_head gyro")
            self.gyro_head.enable(self.timestep)
        self.camera = self.supervisor.getCamera(camera_name)
        self.camera.enable(self.timestep)

        if self.ros_active:
            rospy.init_node("webots_ros_interface", anonymous=True,
                            argv=['clock:=/clock'])
            self.pub_js = rospy.Publisher("joint_states", JointState, queue_size=1)
            self.pub_imu = rospy.Publisher("imu/data", Imu, queue_size=1)

            self.pub_imu_head = rospy.Publisher("imu_head/data", Imu, queue_size=1)
            self.pub_cam = rospy.Publisher("camera/image_proc", Image, queue_size=1)
            self.pub_cam_info = rospy.Publisher("camera/camera_info", CameraInfo, queue_size=1, latch=True)

            self.pub_pres_left = rospy.Publisher("foot_pressure_left/filtered", FootPressure,
                                                 queue_size=1)
            self.pub_pres_right = rospy.Publisher("foot_pressure_right/filtered", FootPressure,
                                                  queue_size=1)
            self.cop_l_pub_ = rospy.Publisher("cop_l", PointStamped, queue_size=1)
            self.cop_r_pub_ = rospy.Publisher("cop_r", PointStamped, queue_size=1)
            self.clock_publisher = rospy.Publisher("/clock", Clock, queue_size=1)
            rospy.Subscriber("DynamixelController/command", JointCommand, self.command_cb)
            self.reset_service = rospy.Service("reset", Empty, self.reset)

        self.translation_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")
        self.world_info = self.supervisor.getFromDef("world_info")


        # publish camera info once, it will be latched
        self.cam_info = CameraInfo()
        self.cam_info.header.stamp = rospy.Time.from_seconds(self.time)
        self.cam_info.header.frame_id = 'camera_optical_frame'
        self.cam_info.height = self.camera.getHeight()
        self.cam_info.width = self.camera.getWidth()
        f_y = self.mat_from_fov_and_resolution(
            self.h_fov_to_v_fov(self.camera.getFov(), self.cam_info.height, self.cam_info.width), 
            self.cam_info.height)
        f_x = self.mat_from_fov_and_resolution(self.camera.getFov(), self.cam_info.width)
        self.cam_info.K = [f_x, 0  , self.cam_info.width / 2,
                      0  , f_x, self.cam_info.height / 2,
                      0  , 0  , 1]
        self.cam_info.P = [f_x, 0, self.cam_info.width / 2  , 0,
                      0  , f_x, self.cam_info.height / 2 , 0,
                      0  , 0  , 1                   , 0]
        if self.ros_active:
            self.pub_cam_info.publish(self.cam_info)
        

    def mat_from_fov_and_resolution(self, fov, res):
        return 0.5 * res * ( math.cos((fov/2)) / math.sin((fov/2)))

    def h_fov_to_v_fov(self, h_fov, height, width):
        return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

    def step_sim(self):
        self.time += self.timestep / 1000
        self.supervisor.step(self.timestep)

    def step(self):
        self.step_sim()
        if self.ros_active:
            self.publish_imu()
            self.publish_joint_states()
            self.publish_camera()
            self.publish_pressure()
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
        return js

    def publish_joint_states(self):
        self.pub_js.publish(self.get_joint_state_msg())

    def get_imu_msg(self, head=False):
        msg = Imu()
        msg.header.stamp = rospy.Time.from_seconds(self.time)
        if head:
            msg.header.frame_id = "imu_frame_2"
        else:
            msg.header.frame_id = "imu_frame"

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

        pos, rpy = self.get_robot_pose_rpy()
        quat_tf = quaternion_from_euler(*rpy)
        msg.orientation = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
        return msg

    def publish_imu(self):
        self.pub_imu.publish(self.get_imu_msg(head=False))
        self.pub_imu_head.publish(self.get_imu_msg(head=True))

    def publish_camera(self):
        img_msg = Image()
        img_msg.header.stamp = rospy.Time.from_seconds(self.time)
        img_msg.header.frame_id = "camera_optical_frame"
        img_msg.height = self.camera.getHeight()
        img_msg.width = self.camera.getWidth()
        img_msg.encoding = "bgra8"
        img_msg.step = 4 * self.camera.getWidth()
        img = self.camera.getImage()
        img_msg.data = img
        self.pub_cam.publish(img_msg)

    def get_image(self):
        return self.camera.getImage()

    def get_pressure_message(self):
        current_time = rospy.Time.from_sec(self.time)

        left_pressure = FootPressure()
        left_pressure.header.stamp = current_time
        left_pressure.left_back = self.pressure_sensors[0].getValues()[2]
        left_pressure.left_front = self.pressure_sensors[1].getValues()[2]
        left_pressure.right_front = self.pressure_sensors[2].getValues()[2]
        left_pressure.right_back = self.pressure_sensors[3].getValues()[2]

        right_pressure = FootPressure()
        left_pressure.header.stamp = current_time
        right_pressure.left_back = self.pressure_sensors[4].getValues()[2]
        right_pressure.left_front = self.pressure_sensors[5].getValues()[2]
        right_pressure.right_front = self.pressure_sensors[6].getValues()[2]
        right_pressure.right_back = self.pressure_sensors[7].getValues()[2]

        # compute center of pressures of the feet
        pos_x = 0.085
        pos_y = 0.045
        # we can take a very small threshold, since simulation gives more accurate values than reality
        threshold = 1

        cop_l = PointStamped()
        cop_l.header.frame_id = "l_sole"
        cop_l.header.stamp = current_time
        sum = left_pressure.left_back + left_pressure.left_front + left_pressure.right_front + left_pressure.right_back
        if sum > threshold:
            cop_l.point.x = (left_pressure.left_front + left_pressure.right_front -
                             left_pressure.left_back - left_pressure.right_back) * pos_x / sum
            cop_l.point.x = max(min(cop_l.point.x, pos_x), -pos_x)
            cop_l.point.y = (left_pressure.left_front + left_pressure.left_back -
                             left_pressure.right_front - left_pressure.right_back) * pos_y / sum
            cop_l.point.y = max(min(cop_l.point.x, pos_y), -pos_y)
        else:
            cop_l.point.x = 0
            cop_l.point.y = 0

        cop_r = PointStamped()
        cop_r.header.frame_id = "r_sole"
        cop_r.header.stamp = current_time
        sum = right_pressure.right_back + right_pressure.right_front + right_pressure.right_front + right_pressure.right_back
        if sum > threshold:
            cop_r.point.x = (right_pressure.left_front + right_pressure.right_front -
                             right_pressure.left_back - right_pressure.right_back) * pos_x / sum
            cop_r.point.x = max(min(cop_r.point.x, pos_x), -pos_x)
            cop_r.point.y = (right_pressure.left_front + right_pressure.left_back -
                             right_pressure.right_front - right_pressure.right_back) * pos_y / sum
            cop_r.point.y = max(min(cop_r.point.x, pos_y), -pos_y)
        else:
            cop_r.point.x = 0
            cop_r.point.y = 0

        return left_pressure, right_pressure, cop_l, cop_r

    def publish_pressure(self):
        left, right, cop_l, cop_r = self.get_pressure_message()
        self.pub_pres_left.publish(left)
        self.pub_pres_right.publish(right)
        self.cop_l_pub_.publish(cop_l)
        self.cop_r_pub_.publish(cop_r)

    def set_gravity(self, active):
        if active:
            self.world_info.getField("gravity").setSFVec3f([0.0, -9.81, 0.0])
            self.world_info.getField("gravity").setSFFloat(9.81)
        else:
            self.world_info.getField("gravity").setSFVec3f([0.0, 0.0, 0.0])
            self.world_info.getField("gravity").setSFFloat(0)

    def reset_robot_pose(self, pos, quat):
        self.set_robot_pose_quat(pos, quat)
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

    def set_robot_axis_angle(self, axis, angle):
        self.rotation_field.setSFRotation(list(np.append(axis,angle)))

    def set_robot_rpy(self, rpy):
        axis, angle = transforms3d.euler.euler2axangle(rpy[0], rpy[1], rpy[2], axes='sxyz')
        self.set_robot_axis_angle(axis,angle)

    def set_robot_quat(self, quat):
        axis, angle = transforms3d.quaternions.quat2axangle([quat[3], quat[0], quat[1], quat[2]])
        self.set_robot_axis_angle(axis, angle)

    def set_robot_position(self, pos):
        self.translation_field.setSFVec3f(list(pos))

    def set_robot_pose_rpy(self, pos, rpy):
        self.set_robot_position(pos)
        self.set_robot_rpy(rpy)

    def set_robot_pose_quat(self, pos, quat):
        self.set_robot_position(pos)
        self.set_robot_quat(quat)

    def get_robot_position(self):
        return self.translation_field.getSFVec3f()
    
    def get_robot_orientation_axangles(self):
        return self.rotation_field.getSFRotation()

    def get_robot_orientation_rpy(self):
        ax_angle = self.get_robot_orientation_axangles()
        return list(transforms3d.euler.axangle2euler(ax_angle[:3], ax_angle[3], axes='sxyz'))
    
    def get_robot_orientation_quat(self):
        ax_angle = self.get_robot_orientation_axangles()
        # transforms 3d uses scalar (i.e. the w part in the quaternion) first notation of quaternions, ros uses scalar last
        quat_scalar_first = transforms3d.quaternions.axangle2quat(ax_angle[:3], ax_angle[3])
        quat_scalar_last = np.append(quat_scalar_first[1:], quat_scalar_first[0])
        return list(quat_scalar_last)

    def get_robot_pose_rpy(self):
        return self.get_robot_position(), self.get_robot_orientation_rpy()

    def get_robot_pose_quat(self):
        return self.get_robot_position(), self.get_robot_orientation_quat()
