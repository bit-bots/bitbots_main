#!/usr/bin/env python3
import math
import sys
import os
import time
import pybullet as p
from time import sleep
from scipy import signal
import pybullet_data
import rospkg


class Simulation:
    def __init__(self, gui):
        self.gui = gui
        self.paused = False
        self.gravity = True

        # config values
        self.start_position = [0, 0, 0.43]
        self.start_orientation = p.getQuaternionFromEuler((0, 0.25, 0))
        self.initial_joints_positions = {"LAnklePitch": -30, "LAnkleRoll": 0, "LHipPitch": 30, "LHipRoll": 0,
                                         "LHipYaw": 0, "LKnee": 60, "RAnklePitch": 30, "RAnkleRoll": 0,
                                         "RHipPitch": -30, "RHipRoll": 0, "RHipYaw": 0, "RKnee": -60,
                                         "LShoulderPitch": 0, "LShoulderRoll": 0, "LElbow": 45, "RShoulderPitch": 0,
                                         "RShoulderRoll": 0, "RElbow": -45, "HeadPan": 0, "HeadTilt": 0}

        # Instantiating Bullet
        if self.gui:
            self.client_id = p.connect(p.GUI)
        else:
            self.client_id = p.connect(p.DIRECT)
        if self.gravity:
            p.setGravity(0, 0, -9.81)
        self.time = 0
        # disable debug interface, only show robot
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)

        # Load floor
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # needed for plane.urdf
        self.plane_index = p.loadURDF('plane.urdf')
        p.changeDynamics(self.plane_index, -1, lateralFriction=1, spinningFriction=-1,
                         rollingFriction=-1, restitution=0.9)

        # Load field
        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path('wolfgang_pybullet_sim'), 'models')
        p.setAdditionalSearchPath(path)  # needed to find field model
        self.field_index = p.loadURDF('field/field.urdf')
        p.changeDynamics(self.field_index, -1, lateralFriction=1, spinningFriction=-1,
                         rollingFriction=-1, restitution=0.9)

        # Load robot
        path = rospack.get_path("wolfgang_description")
        flags = p.URDF_USE_INERTIA_FROM_FILE + p.URDF_USE_SELF_COLLISION + p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        self.robot_index = p.loadURDF(path + "/urdf/robot.urdf",
                                      self.start_position, self.start_orientation, flags=flags)

        # Engine parameters
        # time step should be at 240Hz (due to pyBullet documentation)
        self.timestep = 1 / 240
        # standard parameters seem to be best. leave them like they are
        # p.setPhysicsEngineParameter(fixedTimeStep=self.timestep, numSubSteps=1)
        # no real time, as we will publish own clock
        self.real_time = False
        p.setRealTimeSimulation(0)

        # Retrieving joints and foot pressure sensors
        self.joints = {}
        self.pressure_sensors = {}
        self.links = {}

        # Collecting the available joints
        link_index = 0
        for i in range(p.getNumJoints(self.robot_index)):
            joint_info = p.getJointInfo(self.robot_index, i)
            name = joint_info[1].decode('utf-8')
            # we can get the links by seeing where the joint is attached
            self.links[joint_info[12].decode('utf-8')] = link_index
            link_index += 1
            if name in self.initial_joints_positions.keys():
                # remember joint
                self.joints[name] = Joint(i, self.robot_index)
            elif name in ["LLB", "LLF", "LRF", "LRB", "RLB", "RLF", "RRF", "RRB"]:
                p.enableJointForceTorqueSensor(self.robot_index, i)
                self.pressure_sensors[name] = PressureSensor(name, i, self.robot_index, 10, 5)

        # set friction for feet
        self.set_foot_dynamics(0.0, 0.0, 0.0)

        # reset robot to initial position
        self.reset()

    def set_foot_dynamics(self, contact_damping, contact_stiffness, joint_damping, lateral_friction=1,
                          spinning_friction=0, rolling_friction=0):
        for link_name in self.links.keys():
            if link_name in ["llb", "llf", "lrf", "lrb", "rlb", "rlf", "rrf", "rrb"]:
                # print(p.getLinkState(self.robot_index, self.links[link_name]))
                p.changeDynamics(self.robot_index, self.links[link_name],
                                 lateralFriction=lateral_friction,
                                 spinningFriction=spinning_friction,
                                 rollingFriction=rolling_friction,
                                 contactDamping=contact_damping,
                                 contactStiffness=contact_stiffness,
                                 jointDamping=joint_damping)
                p.changeDynamics(self.plane_index, -1, lateralFriction=lateral_friction,
                                 spinningFriction=spinning_friction,
                                 rollingFriction=rolling_friction, restitution=0.9)

    def set_filter_params(self, cutoff, order):
        for i in range(p.getNumJoints(self.robot_index)):
            joint_info = p.getJointInfo(self.robot_index, i)
            name = joint_info[1].decode('utf-8')
            if name in ["LLB", "LLF", "LRF", "LRB", "RLB", "RLF", "RRF", "RRB"]:
                self.pressure_sensors[name] = PressureSensor(name, i, self.robot_index, cutoff, order)

    def reset(self):
        # set joints to initial position
        for name in self.joints:
            joint = self.joints[name]
            pos_in_rad = math.radians(self.initial_joints_positions[name])
            joint.reset_position(pos_in_rad, 0)
            joint.set_position(pos_in_rad)

        # reset body pose and velocity
        p.resetBasePositionAndOrientation(self.robot_index, self.start_position, self.start_orientation)
        p.resetBaseVelocity(self.robot_index, [0, 0, 0], [0, 0, 0])

    def step(self):
        # get keyboard events if gui is active
        single_step = False
        if self.gui:
            # rest if R-key was pressed
            rKey = ord('r')
            nKey = ord('n')
            sKey = ord('s')
            tKey = ord('t')
            spaceKey = p.B3G_SPACE
            keys = p.getKeyboardEvents()
            if rKey in keys and keys[rKey] & p.KEY_WAS_TRIGGERED:
                self.reset()
            if spaceKey in keys and keys[spaceKey] & p.KEY_WAS_TRIGGERED:
                self.paused = not self.paused
            if sKey in keys and keys[sKey] & p.KEY_IS_DOWN:
                single_step = True
            if nKey in keys and keys[nKey] & p.KEY_WAS_TRIGGERED:
                self.set_gravity(not self.gravity)
            if tKey in keys and keys[tKey] & p.KEY_WAS_TRIGGERED:
                self.real_time = not self.real_time
                p.setRealTimeSimulation(self.real_time)

        # check if simulation should continue currently
        if not self.paused or single_step:
            self.time += self.timestep
            p.stepSimulation()
            for name, ps in self.pressure_sensors.items():
                ps.filter_step()

    def set_gravity(self, active):
        if active:
            p.setGravity(0, 0, -9.81)
        else:
            p.setGravity(0, 0, 0)
        self.gravity = active

    def reset_robot_pose(self, position, orientation):
        # reset body pose and velocity
        p.resetBasePositionAndOrientation(self.robot_index, position, orientation)
        p.resetBaseVelocity(self.robot_index, [0, 0, 0], [0, 0, 0])

    def get_robot_pose(self):
        (x, y, z), (qx, qy, qz, qw) = p.getBasePositionAndOrientation(self.robot_index)
        return (x, y, z), (qx, qy, qz, qw)

    def get_robot_pose_rpy(self):
        (x, y, z), (qx, qy, qz, qw) = p.getBasePositionAndOrientation(self.robot_index)
        (roll, pitch, yaw) = p.getEulerFromQuaternion((qx, qy, qz, qw))
        return (x, y, z), (roll, pitch, yaw)

    def get_robot_velocity(self):
        (vx, vy, vz), (vr, vp, vy) = p.getBaseVelocity(self.robot_index)
        return (vx, vy, vz), (vr, vp, vy)


class Joint:
    def __init__(self, joint_index, body_index):
        self.joint_index = joint_index
        self.body_index = body_index
        joint_info = p.getJointInfo(self.body_index, self.joint_index)
        self.name = joint_info[1].decode('utf-8')
        self.type = joint_info[2]
        self.max_force = joint_info[10]
        self.max_velocity = joint_info[11]
        self.lowerLimit = joint_info[8]
        self.upperLimit = joint_info[9]
        self.damping = joint_info[6]
        self.friction = joint_info[7]

    def reset_position(self, position, velocity):
        p.resetJointState(self.body_index, self.joint_index, targetValue=position, targetVelocity=velocity)
        # self.disable_motor()

    def disable_motor(self):
        p.setJointMotorControl2(self.body_index, self.joint_index,
                                controlMode=p.POSITION_CONTROL, targetPosition=0, targetVelocity=0,
                                positionGain=0.1, velocityGain=0.1, force=0)

    def set_position(self, position):
        p.setJointMotorControl2(self.body_index, self.joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=position, force=self.max_force - self.friction,
                                maxVelocity=self.max_velocity)

    def get_state(self):
        position, velocity, forces, applied_torque = p.getJointState(self.body_index, self.joint_index)
        return position, velocity, forces, applied_torque

    def get_position(self):
        position, velocity, forces, applied_torque = self.get_state()
        return position

    def get_velocity(self):
        position, velocity, forces, applied_torque = self.get_state()
        return velocity

    def get_torque(self):
        position, velocity, forces, applied_torque = self.get_state()
        return applied_torque


class PressureSensor:
    def __init__(self, name, joint_index, body_index, cutoff, order):
        self.joint_index = joint_index
        self.name = name
        self.body_index = body_index
        nyq = 240 * 0.5  # nyquist frequency from simulation frequency
        normalized_cutoff = cutoff / nyq  # cutoff freq in hz
        self.filter_b, self.filter_a = signal.butter(order, normalized_cutoff, btype='low')
        self.filter_state = signal.lfilter_zi(self.filter_b, 1)
        self.unfiltered = 0
        self.filtered = [0]

    def filter_step(self):
        self.unfiltered = p.getJointState(self.body_index, self.joint_index)[2][2] * -1
        self.filtered, self.filter_state = signal.lfilter(self.filter_b, self.filter_a, [self.unfiltered],
                                                          zi=self.filter_state)

    def get_force(self):
        return max(self.unfiltered, 0), max(self.filtered[0], 0)
