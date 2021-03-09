from controller import Robot, Node, Field

import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo

from bitbots_msgs.msg import JointCommand, FootPressure
import math


class RobotController:
    def __init__(self, ros_active=False, robot='wolfgang', do_ros_init=True, external_controller=False, base_ns=''):
        """
        The RobotController, a Webots controller that controls a single robot.
        The environment variable WEBOTS_ROBOT_NAME should be set to "amy", "rory", "jack" or "donna" if used with
        4_bots.wbt or to "amy" if used with 1_bot.wbt.

        :param ros_active: Whether ROS messages should be published
        :param robot: The name of the robot to use, currently one of wolfgang, darwin, nao, op3
        :param do_ros_init: Whether to call rospy.init_node (only used when ros_active is True)
        :param external_controller: Whether an external controller is used, necessary for RobotSupervisorController
        :param base_ns: The namespace of this node, can normally be left empty
        """
        self.ros_active = ros_active
        if not external_controller:
            self.robot_node = Robot()
        self.walkready = [0] * 20
        self.time = 0

        self.motors = []
        self.sensors = []
        self.timestep = int(self.robot_node.getBasicTimeStep())

        self.switch_coordinate_system = True
        self.is_wolfgang = False
        self.pressure_sensors = None
        if robot == 'wolfgang':
            self.is_wolfgang = True
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
                sensor = self.robot_node.getDevice(name)
                sensor.enable(30)
                self.pressure_sensors.append(sensor)

        elif robot == 'darwin':
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

        # self.robot_node = self.supervisor.getFromDef(self.robot_node_name)
        for motor_name in self.motor_names:
            self.motors.append(self.robot_node.getDevice(motor_name))
            self.motors[-1].enableTorqueFeedback(self.timestep)
            self.sensors.append(self.robot_node.getDevice(motor_name + sensor_postfix))
            self.sensors[-1].enable(self.timestep)

        self.accel = self.robot_node.getDevice(accel_name)
        self.accel.enable(self.timestep)
        self.gyro = self.robot_node.getDevice(gyro_name)
        self.gyro.enable(self.timestep)
        if self.is_wolfgang:
            self.accel_head = self.robot_node.getDevice("imu_head accelerometer")
            self.accel_head.enable(self.timestep)
            self.gyro_head = self.robot_node.getDevice("imu_head gyro")
            self.gyro_head.enable(self.timestep)
        self.camera = self.robot_node.getDevice(camera_name)
        self.camera.enable(self.timestep)

        if self.ros_active:
            if base_ns == "":
                clock_topic = "/clock"
            else:
                clock_topic = base_ns + "clock"
            if do_ros_init:
                rospy.init_node("webots_ros_interface", argv=['clock:=' + clock_topic])
            self.l_sole_frame = rospy.get_param("~l_sole_frame", "l_sole")
            self.r_sole_frame = rospy.get_param("~r_sole_frame", "r_sole")
            self.camera_optical_frame = rospy.get_param("~camera_optical_frame", "camera_optical_frame")
            self.head_imu_frame = rospy.get_param("~head_imu_frame", "imu_frame_2")
            self.imu_frame = rospy.get_param("~imu_frame", "imu_frame")
            self.pub_js = rospy.Publisher(base_ns + "joint_states", JointState, queue_size=1)
            self.pub_imu = rospy.Publisher(base_ns + "imu/data_raw", Imu, queue_size=1)

            self.pub_imu_head = rospy.Publisher(base_ns + "imu_head/data", Imu, queue_size=1)
            self.pub_cam = rospy.Publisher(base_ns + "camera/image_proc", Image, queue_size=1)
            self.pub_cam_info = rospy.Publisher(base_ns + "camera/camera_info", CameraInfo, queue_size=1, latch=True)

            self.pub_pres_left = rospy.Publisher(base_ns + "foot_pressure_left/filtered", FootPressure, queue_size=1)
            self.pub_pres_right = rospy.Publisher(base_ns + "foot_pressure_right/filtered", FootPressure, queue_size=1)
            self.cop_l_pub_ = rospy.Publisher(base_ns + "cop_l", PointStamped, queue_size=1)
            self.cop_r_pub_ = rospy.Publisher(base_ns + "cop_r", PointStamped, queue_size=1)
            rospy.Subscriber(base_ns + "DynamixelController/command", JointCommand, self.command_cb)

            # publish camera info once, it will be latched
            self.cam_info = CameraInfo()
            self.cam_info.header.stamp = rospy.Time.from_seconds(self.time)
            self.cam_info.header.frame_id = self.camera_optical_frame
            self.cam_info.height = self.camera.getHeight()
            self.cam_info.width = self.camera.getWidth()
            f_y = self.mat_from_fov_and_resolution(
                self.h_fov_to_v_fov(self.camera.getFov(), self.cam_info.height, self.cam_info.width),
                self.cam_info.height)
            f_x = self.mat_from_fov_and_resolution(self.camera.getFov(), self.cam_info.width)
            self.cam_info.K = [f_x, 0, self.cam_info.width / 2,
                               0, f_y, self.cam_info.height / 2,
                               0, 0, 1]
            self.cam_info.P = [f_x, 0, self.cam_info.width / 2, 0,
                               0, f_y, self.cam_info.height / 2, 0,
                               0, 0, 1, 0]
            self.pub_cam_info.publish(self.cam_info)

    def mat_from_fov_and_resolution(self, fov, res):
        return 0.5 * res * (math.cos((fov / 2)) / math.sin((fov / 2)))

    def h_fov_to_v_fov(self, h_fov, height, width):
        return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

    def step_sim(self):
        self.time += self.timestep / 1000
        self.robot_node.step(self.timestep)

    def step(self):
        self.step_sim()
        if self.ros_active:
            self.publish_ros()

    def publish_ros(self):
        self.publish_imu()
        self.publish_joint_states()
        self.publish_camera()
        self.publish_pressure()

    def command_cb(self, command: JointCommand):
        for i, name in enumerate(command.joint_names):
            try:
                motor_index = self.external_motor_names.index(name)
                self.motors[motor_index].setPosition(command.positions[i])
                if command.velocities[i] == -1:
                    self.motors[motor_index].setVelocity(self.motors[motor_index].getMaxVelocity())
                else:
                    self.motors[motor_index].setVelocity(command.velocities[i])
                self.motors[motor_index].setAcceleration(command.accelerations[i])

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

    def publish_imu(self):
        self.pub_imu.publish(self.get_imu_msg(head=False))
        self.pub_imu_head.publish(self.get_imu_msg(head=True))

    def publish_camera(self):
        img_msg = Image()
        img_msg.header.stamp = rospy.Time.from_seconds(self.time)
        img_msg.header.frame_id = self.camera_optical_frame
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
        cop_l.header.frame_id = self.l_sole_frame
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
        cop_r.header.frame_id = self.r_sole_frame
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
