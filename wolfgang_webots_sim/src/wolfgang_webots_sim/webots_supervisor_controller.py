from controller import Robot, Node, Supervisor, Field

import os
import rospy
from geometry_msgs.msg import Quaternion, PointStamped
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo

from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty

from bitbots_msgs.msg import JointCommand, FootPressure
import math
import transforms3d
import numpy as np

G = 9.81


class SupervisorController:
    def __init__(self, robot_name='', ros_active=False, mode='normal', robot='wolfgang'):
        # requires WEBOTS_ROBOT_NAME to be set to "amy" or "rory"
        self.ros_active = ros_active
        self.time = 0
        self.clock_msg = Clock()

        self.supervisor = Supervisor()
        self.amy_node = self.supervisor.getFromDef("amy")
        self.rory_node = self.supervisor.getFromDef("rory")
        self.jack_node = self.supervisor.getFromDef("jack")
        self.donna_node = self.supervisor.getFromDef("donna")
        self.melody_node = self.supervisor.getFromDef("melody")

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

        self.motors = []
        self.sensors = []
        self.timestep = int(self.supervisor.getBasicTimeStep())

        # resolve the node for corresponding name
        self.robot_nodes = {"amy": self.amy_node, "rory": self.rory_node, "jack": self.jack_node, "donna": self.donna_node, "melody": self.melody_node}

        self.robot_name = robot_name
        self.switch_coordinate_system = True
        self.is_wolfgang = False
        self.pressure_sensors = None

        if self.ros_active:
            rospy.init_node("webots_ros_supervisor", anonymous=True,
                            argv=['clock:=/clock'])
            self.clock_publisher = rospy.Publisher("/clock", Clock, queue_size=1)
            self.reset_service = rospy.Service("reset", Empty, self.reset)

        self.translation_field = self.name_to_node().getField("translation")
        self.rotation_field = self.name_to_node().getField("rotation")
        self.world_info = self.supervisor.getFromDef("world_info")

    def name_to_node(self):
        return self.robot_nodes[os.environ["WEBOTS_ROBOT_NAME"]]


    def step_sim(self):
        self.time += self.timestep / 1000
        self.name_to_node().step(self.timestep)

    def step(self):
        self.step_sim()
        if self.ros_active:
            self.publish_clock()

    def publish_clock(self):
        self.clock_msg.clock = rospy.Time.from_seconds(self.time)
        self.clock_publisher.publish(self.clock_msg)

    def set_gravity(self, active):
        if active:
            self.world_info.getField("gravity").setSFVec3f([0.0, -9.81, 0.0])
            self.world_info.getField("gravity").setSFFloat(9.81)
        else:
            self.world_info.getField("gravity").setSFVec3f([0.0, 0.0, 0.0])
            self.world_info.getField("gravity").setSFFloat(0)

    def reset_robot_pose(self, pos, quat):
        self.set_robot_pose_quat(pos, quat)
        self.name_to_node().resetPhysics()

    def reset_robot_pose_rpy(self, pos, rpy):
        self.set_robot_pose_rpy(pos, rpy)
        self.name_to_node().resetPhysics()

    def reset(self, req=None):
        self.supervisor.simulationReset()
        self.supervisor.simulationResetPhysics()

    def node(self):
        s = self.supervisor.getSelected()
        if s is not None:
            print(f"id: {s.getId()}, type: {s.getType()}, def: {s.getDef()}")

    def set_robot_axis_angle(self, axis, angle):
        self.rotation_field.setSFRotation(list(np.append(axis, angle)))

    def set_robot_rpy(self, rpy):
        axis, angle = transforms3d.euler.euler2axangle(rpy[0], rpy[1], rpy[2], axes='sxyz')
        self.set_robot_axis_angle(axis, angle)

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
