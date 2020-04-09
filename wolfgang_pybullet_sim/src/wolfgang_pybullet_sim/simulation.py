#!/usr/bin/env python3
import math
import sys
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
                                         "LHipYaw": 0, "LKnee": -60, "RAnklePitch": 30, "RAnkleRoll": 0,
                                         "RHipPitch": -30, "RHipRoll": 0, "RHipYaw": 0, "RKnee": 60,
                                         "LShoulderPitch": 0, "LShoulderRoll": 0, "LElbow": 45, "RShoulderPitch": 0,
                                         "RShoulderRoll": 0, "RElbow": -45, "HeadPan": 0, "HeadTilt": 0}


        # Instantiating Bullet
        if self.gui:
            self.client_id = p.connect(p.GUI)
        else:
            self.client_id = p.connect(p.SHARED_MEMORY_SERVER)
        p.setGravity(0, 0, -9.81)
        self.time = 0
        # disable debug interface, only show robot
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)

        # Loading floor
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        self.plane_index = p.loadURDF('plane.urdf')
        p.changeDynamics(self.plane_index, -1, lateralFriction=100000000000000, spinningFriction=100000000000,
                         rollingFriction=0.1, restitution=0.9)

        # Loading robot
        rospack = rospkg.RosPack()
        path = rospack.get_path("wolfgang_description")
        flags = p.URDF_USE_INERTIA_FROM_FILE
        self.robot_index = p.loadURDF(path + "/urdf/robot.urdf",
                                      self.start_position, self.start_orientation, flags=flags)

        # Engine parameters
        # time step should be at 240Hz (due to pyBullet documentation)
        self.timestep = 1/240
        # standard parameters seem to be best. leave them like they are
        # p.setPhysicsEngineParameter(fixedTimeStep=self.timestep, numSubSteps=1)
        # no real time, as we will publish own clock
        p.setRealTimeSimulation(0)

        # Retrieving joints and foot pressure sensors
        self.joints = {}
        self.pressure_sensors = {}
        self.links = {}

        # Collecting the available joints
        for i in range(p.getNumJoints(self.robot_index)):
            joint_info = p.getJointInfo(self.robot_index, i)
            name = joint_info[1].decode('utf-8')
            # we can get the links by seeing where the joint is attached
            self.links[joint_info[12].decode('utf-8')] = joint_info[16]
            if name in self.initial_joints_positions.keys():
                # remember joint
                self.joints[name] = Joint(i, self.robot_index)
            elif name in ["LLB", "LLF", "LRF", "LRB", "RLB", "RLF", "RRF", "RRB"]:
                self.pressure_sensors[name] = PressureSensor(name, i, self.robot_index)

        # set friction for feet
        self.set_foot_dynamics(0.5,0.5)

        # reset robot to initial position
        self.reset()

    def set_foot_dynamics(self, contact_damping, contact_stiffness):
        for link_name in self.links.keys():
            if link_name in ["l_foot", "r_foot", "llb", "llf", "lrf", "lrb", "rlb", "rlf", "rrf", "rrb"]:
                # print(self.parts[part].body_name)
                p.changeDynamics(self.robot_index, self.links[link_name],
                                 lateralFriction=10000000000000000,
                                 spinningFriction=1000000000000000,
                                 rollingFriction=0.1,
                                 contactDamping=contact_damping,
                                 contactStiffness=contact_stiffness)
        print("contact_stiffness: " + str(contact_stiffness))
        print("contact_damping: " + str(contact_damping))

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
            spaceKey = p.B3G_SPACE
            keys = p.getKeyboardEvents()
            if rKey in keys and keys[rKey] & p.KEY_WAS_TRIGGERED:
                self.reset()
            if spaceKey in keys and keys[spaceKey] & p.KEY_WAS_TRIGGERED:
                self.paused = not self.paused
            if sKey in keys and keys[sKey] & p.KEY_IS_DOWN:
                single_step = True
            if nKey in keys and keys[nKey] & p.KEY_WAS_TRIGGERED:
                if self.gravity:
                    p.setGravity(0, 0, 0)
                else:
                    p.setGravity(0, 0, -9.81)
                self.gravity = not self.gravity

        # check if simulation should continue currently
        if not self.paused or single_step:
            self.time += self.timestep
            p.stepSimulation()

    def get_robot_pose(self):
        (x, y, z), (qx, qy, qz, qw) = p.getBasePositionAndOrientation(self.robot_index)
        return (x, y, z), (qx, qy, qz, qw)

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

    def reset_position(self, position, velocity):
        p.resetJointState(self.body_index, self.joint_index, targetValue=position, targetVelocity=velocity)
        #self.disable_motor()

    def disable_motor(self):
        p.setJointMotorControl2(self.body_index, self.joint_index,
                                controlMode=p.POSITION_CONTROL, targetPosition=0, targetVelocity=0,
                                positionGain=0.1, velocityGain=0.1, force=0)

    def set_position(self, position):
        p.setJointMotorControl2(self.body_index, self.joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=position, force=self.max_force,
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
    def __init__(self, name, joint_index, body_index):
        self.joint_index = joint_index
        self.name = name
        self.body_index = body_index
        p.enableJointForceTorqueSensor(self.body_index, self.joint_index)
        order = 5
        nyq = 240 * 0.5 # nyquist frequency from simulation frequency
        normalized_cutoff = 10 / nyq # cutoff freq in hz
        self.filter_b, self.filter_a = signal.butter(order, normalized_cutoff, btype='low')
        self.filter_state = signal.lfilter_zi(self.filter_b, 1)

    def get_force(self):
        unfiltered = p.getJointState(self.body_index, self.joint_index)[2][2] * -1
        filtered, self.filter_state = signal.lfilter(self.filter_b, self.filter_a, [unfiltered], zi=self.filter_state)
        return unfiltered, filtered
