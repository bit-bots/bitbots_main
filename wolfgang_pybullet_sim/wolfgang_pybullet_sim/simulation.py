#!/usr/bin/env python3
import math
import random
import sys
import os

import pybullet as p
from time import sleep, time

import rclpy
import tf_transformations
import tf2_py
from scipy import signal
import pybullet_data
import rospkg
from ament_index_python import get_package_share_directory
from transforms3d.euler import quat2euler, euler2quat
from transforms3d.quaternions import quat2mat, rotate_vector, qinverse

from wolfgang_pybullet_sim.terrain import Terrain
import numpy as np


class Simulation:
    def __init__(self, gui, urdf_path=None, foot_link_names=[], terrain_height=0, field=False, joints_ft=False,
                 robot="wolfgang", load_robot=True):
        self.gui = gui
        self.paused = False
        self.gravity = True
        self.terrain_height = terrain_height
        self.field_on = field
        self.urdf_path = urdf_path
        self.foot_link_names = foot_link_names
        self.joints_ft = joints_ft
        self.robot_type = robot
        self.load_robot = load_robot
        self.robot_indexes = []
        self.joints = {}
        self.joint_stall_torques = {}
        self.joint_max_vels = {}
        self.link_masses = {}
        self.link_inertias = {}
        self.pressure_sensors = {}
        self.links = {}
        self.torso_ids = {}
        self.last_step_time = 0
        self.realtime = False
        self.time_multiplier = 0

        # config values
        self.start_position = [0, 0, 0.43]
        self.start_orientation = p.getQuaternionFromEuler((0, 0.25, 0))
        if self.robot_type is None:
            # no robot to initialize
            pass
        elif self.robot_type == "wolfgang":
            self.initial_joint_positions = {"LAnklePitch": -30, "LAnkleRoll": 0, "LHipPitch": 30, "LHipRoll": 0,
                                            "LHipYaw": 0, "LKnee": 60, "RAnklePitch": 30, "RAnkleRoll": 0,
                                            "RHipPitch": -30, "RHipRoll": 0, "RHipYaw": 0, "RKnee": -60,
                                            "LShoulderPitch": 75, "LShoulderRoll": 0, "LElbow": 36,
                                            "RShoulderPitch": -75, "RShoulderRoll": 0, "RElbow": -36, "HeadPan": 0,
                                            "HeadTilt": 0}
        elif self.robot_type in ["op2", "robotis_op2"]:
            self.initial_joint_positions = {"l_ankle_pitch": 0, "l_ankle_roll": 0, "l_hip_pitch": 0, "l_hip_roll": 0,
                                            "l_hip_yaw": 0, "l_knee": 0, "r_ankle_pitch": 0, "r_ankle_roll": 0,
                                            "r_hip_pitch": 0, "r_hip_roll": 0, "r_hip_yaw": 0, "r_knee": 0,
                                            "l_sho_pitch": 0, "l_sho_roll": 0, "LElbow": 0, "r_sho_pitch": 0,
                                            "r_sho_roll": 0, "RElbow": 0, "head_pan": 0, "head_tilt": 0}
            self.foot_link_names = ["l_foot", "r_foot"]
        elif self.robot_type == "sigmaban":
            self.start_orientation = p.getQuaternionFromEuler((0, 0.0, 0))
            self.initial_joint_positions = {"left_ankle_pitch": 0, "left_ankle_roll": 0, "left_hip_pitch": 0,
                                            "left_hip_roll": 0, "left_hip_yaw": 0, "left_knee": 0,
                                            "right_ankle_pitch": 0, "right_ankle_roll": 0, "right_hip_pitch": 0,
                                            "right_hip_roll": 0, "right_hip_yaw": 0, "right_knee": 0,
                                            "left_shoulder_pitch": 0, "left_shoulder_roll": 0, "LElbow": 0,
                                            "right_shoulder_pitch": 0, "right_shoulder_roll": 0, "RElbow": 0,
                                            "head_yaw": 0, "head_pitch": 0}
        else:
            print(f"robot {self.robot_type} not known")
            quit(0)
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

        # Engine parameters
        # time step should be at 240Hz (due to pyBullet documentation)
        self.timestep = 1 / 240
        # standard parameters seem to be best. leave them like they are
        # p.setPhysicsEngineParameter(fixedTimeStep=self.timestep, numSubSteps=1)
        # no real time, as we will publish own clock, but we have option to switch to real_time by waiting
        p.setRealTimeSimulation(0)
        self.last_wall_time = time()

        self.load_models()

    def load_models(self):
        # Load floor
        self.terrain_index = None
        self.plane_index = None
        if self.terrain_height > 0:
            self.terrain = Terrain(self.terrain_height, clear_center=False)
            self.terrain_index = self.terrain.id
            p.changeDynamics(self.terrain_index, -1, lateralFriction=1, spinningFriction=0.1, rollingFriction=0.1,
                             restitution=0.9)
        else:
            # Loading floor
            p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
            self.plane_index = p.loadURDF('plane.urdf')
            p.changeDynamics(self.plane_index, -1, lateralFriction=1, spinningFriction=0.1, rollingFriction=0.1,
                             restitution=0.9)

        self.field_index = None
        if self.field_on:
            # Load field

            rospack = rospkg.RosPack()
            path = os.path.join(rospack.get_path('wolfgang_pybullet_sim'), 'models')
            p.setAdditionalSearchPath(path)  # needed to find field model
            self.field_index = p.loadURDF('field/field.urdf')

        if self.load_robot:
            self.add_robot()

    def add_robot(self, physics_active=True):
        if not physics_active:
            flags = p.URDF_USE_INERTIA_FROM_FILE
        elif self.robot_type in ["wolfgang", "sigmaban"]:
            # we have a better inertia estimation from onshape in our model
            flags = p.URDF_USE_SELF_COLLISION + p.URDF_USE_INERTIA_FROM_FILE + \
                    p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
        else:
            # most other URDFs from the internet have issues with their inertia values
            flags = p.URDF_USE_SELF_COLLISION + p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS

        if self.urdf_path is None:
            # use standards
            if self.robot_type == "op2":
                self.urdf_path = get_package_share_directory("robotis_op2_description") + "/urdf/robot.urdf"
            elif self.robot_type == "sigmaban":
                self.urdf_path = get_package_share_directory("sigmaban_description") + "/urdf/robot.urdf"
            else:
                self.urdf_path = get_package_share_directory("wolfgang_description") + "/urdf/robot.urdf"
        robot_index = p.loadURDF(self.urdf_path, self.start_position, self.start_orientation, flags=flags,
                                 useFixedBase=not physics_active)
        self.robot_indexes.append(robot_index)

        # disable physics for robots that dont need it (mostly used to display reference trajectories)
        if not physics_active:
            p.changeDynamics(robot_index, -1, linearDamping=0, angularDamping=0)
            p.setCollisionFilterGroupMask(robot_index, -1, collisionFilterGroup=0,
                                          collisionFilterMask=0)
            p.changeDynamics(robot_index, -1, activationState=p.ACTIVATION_STATE_SLEEP +
                                                              p.ACTIVATION_STATE_ENABLE_SLEEPING +
                                                              p.ACTIVATION_STATE_DISABLE_WAKEUP)
            num_joints = p.getNumJoints(robot_index)
            for j in range(num_joints):
                p.setCollisionFilterGroupMask(robot_index, j, collisionFilterGroup=0,
                                              collisionFilterMask=0)
                p.changeDynamics(robot_index, j, activationState=p.ACTIVATION_STATE_SLEEP +
                                                                 p.ACTIVATION_STATE_ENABLE_SLEEPING +
                                                                 p.ACTIVATION_STATE_DISABLE_WAKEUP)

        # Retrieving joints and foot pressure sensors
        joints = {}
        joint_stall_torques = []
        joint_max_vels = []
        link_masses = []
        link_inertias = []
        pressure_sensors = {}
        links = {}

        # Collecting the available joints
        for i in range(p.getNumJoints(robot_index)):
            joint_info = p.getJointInfo(robot_index, i)
            name = joint_info[1].decode('utf-8')
            # we can get the links by seeing where the joint is attached. we only get parent link so do +1
            links[joint_info[12].decode('utf-8')] = joint_info[16] + 1
            if name in self.initial_joint_positions.keys():
                # remember joint
                joints[name] = Joint(i, robot_index)
                joint_stall_torques.append(joint_info[10])
                joint_max_vels.append(joint_info[11])
            elif name in ["LLB", "LLF", "LRF", "LRB", "RLB", "RLF", "RRF", "RRB"]:
                p.enableJointForceTorqueSensor(robot_index, i)
                pressure_sensors[name] = PressureSensor(name, i, robot_index, 10, 5)

        for link_id in links.values():
            dynamics_info = p.getDynamicsInfo(robot_index, link_id)
            link_masses.append(dynamics_info[0])
            link_inertias.append(dynamics_info[2])

        # set dynamics for feet, maybe overwritten later by domain randomization
        for link_name in links.keys():
            if link_name in ["llb", "llf", "lrf", "lrb", "rlb", "rlf", "rrf", "rrb"]:
                p.changeDynamics(robot_index, links[link_name], lateralFriction=1, spinningFriction=0.1,
                                 rollingFriction=0.1, restitution=0.9)

        self.joints[robot_index] = joints
        self.joint_stall_torques[robot_index] = joint_stall_torques
        self.joint_max_vels[robot_index] = joint_max_vels
        self.link_masses[robot_index] = link_masses
        self.link_inertias[robot_index] = link_inertias
        self.pressure_sensors[robot_index] = pressure_sensors
        self.links[robot_index] = links

        self.torso_ids[robot_index] = self.links[robot_index]["torso"]

        # reset robot to initial position
        self.reset(robot_index)

        return robot_index

    def apply_force(self, link_id, force, position, robot_index=1):
        """
        Applies an external force to a position on a link.
        :param link_id: link index or -1 for base link (int)
        :param force: direction and amount of applied force (vec3)
        :param position: where on the link the force should be applied (vec3)
        """
        p.applyExternalForce(robot_index, link_id, force, position, flags=p.WORLD_FRAME)

    def set_dynamics(self, contact_damping=-1.0, contact_stiffness=-1.0, lateral_friction=1,
                          spinning_friction=1, rolling_friction=1, restitution=0, robot_index=1):
        # set dynamic values for all links and ground
        for link_name in self.links[robot_index].keys():
            if link_name in ["llb", "llf", "lrf", "lrb", "rlb", "rlf", "rrf",
                             "rrb"] or link_name in self.foot_link_names:
                # print(p.getLinkState(self.robot_type_index, self.links[link_name]))
                p.changeDynamics(robot_index, self.links[robot_index][link_name],
                                 lateralFriction=lateral_friction,
                                 spinningFriction=spinning_friction, rollingFriction=rolling_friction, restitution=restitution,
                                 contactDamping=contact_damping, contactStiffness=contact_stiffness)
        if self.plane_index:
            p.changeDynamics(self.plane_index, -1, lateralFriction=lateral_friction,
                             spinningFriction=spinning_friction, rollingFriction=rolling_friction, restitution=restitution,
                                 contactDamping=contact_damping, contactStiffness=contact_stiffness)
        if self.field_index:
            p.changeDynamics(self.field_index, -1, lateralFriction=lateral_friction,
                             spinningFriction=spinning_friction, rollingFriction=rolling_friction, restitution=restitution,
                                 contactDamping=contact_damping, contactStiffness=contact_stiffness)
        if self.terrain_index:
            p.changeDynamics(self.terrain_index, -1, lateralFriction=lateral_friction,
                             spinningFriction=spinning_friction, rollingFriction=rolling_friction, restitution=restitution,
                                 contactDamping=contact_damping, contactStiffness=contact_stiffness)

    def randomize_links(self, mass_bounds, inertia_bounds, robot_index=1):
        i = 0
        for link_id in self.links[robot_index].values():
            randomized_mass = random.uniform(mass_bounds[0], mass_bounds[1]) * self.link_masses[robot_index][i]
            randomized_inertia = random.uniform(inertia_bounds[0], inertia_bounds[1]) * np.array(
                self.link_inertias[robot_index][i])
            p.changeDynamics(robot_index, link_id, mass=randomized_mass,
                             localInertiaDiagonal=randomized_inertia)
            i += 1

    def randomize_joints(self, torque_bounds, vel_bounds, robot_index=1):
        i = 0
        for joint_name in self.joints[robot_index].keys():
            randomized_stall_torque = random.uniform(torque_bounds[0], torque_bounds[1]) * \
                                      self.joint_stall_torques[robot_index][i]
            self.joints[robot_index][joint_name].max_torque = randomized_stall_torque
            randomized_max_vel = random.uniform(vel_bounds[0], vel_bounds[1]) * self.joint_max_vels[robot_index][i]
            self.joints[robot_index][joint_name].max_vel = randomized_max_vel

    def randomize_foot_friction(self, restitution_bounds, lateral_friction_bounds, spinning_friction_bounds,
                                rolling_friction_bounds, robot_index=1):
        # set dynamic values for all foot links
        rand_restitution = random.uniform(restitution_bounds[0], restitution_bounds[1])
        rand_lateral_friction = random.uniform(lateral_friction_bounds[0], lateral_friction_bounds[1])
        rand_spinning_friction = random.uniform(spinning_friction_bounds[0], spinning_friction_bounds[1])
        rand_rolling_friction = random.uniform(rolling_friction_bounds[0], rolling_friction_bounds[1])

        for link_name in self.links[robot_index].keys():
            if link_name in ["llb", "llf", "lrf", "lrb", "rlb", "rlf", "rrf", "rrb"]:
                p.changeDynamics(robot_index, self.links[robot_index][link_name],
                                 lateralFriction=rand_lateral_friction,
                                 spinningFriction=rand_spinning_friction,
                                 rollingFriction=rand_rolling_friction,
                                 restitution=rand_restitution)

    def randomize_floor(self, restitution_bounds, lateral_friction_bounds, spinning_friction_bounds,
                        rolling_friction_bounds):
        # set dynamic values for the ground
        rand_restitution = random.uniform(restitution_bounds[0], restitution_bounds[1])
        rand_lateral_friction = random.uniform(lateral_friction_bounds[0], lateral_friction_bounds[1])
        rand_spinning_friction = random.uniform(spinning_friction_bounds[0], spinning_friction_bounds[1])
        rand_rolling_friction = random.uniform(rolling_friction_bounds[0], rolling_friction_bounds[1])

        if self.terrain_height > 0:
            p.changeDynamics(self.terrain_index, -1,
                             lateralFriction=rand_lateral_friction,
                             spinningFriction=rand_spinning_friction,
                             rollingFriction=rand_rolling_friction,
                             restitution=rand_restitution)
        else:
            p.changeDynamics(self.plane_index, -1,
                             lateralFriction=rand_lateral_friction,
                             spinningFriction=rand_spinning_friction,
                             rollingFriction=rand_rolling_friction,
                             restitution=rand_restitution)

    def randomize_terrain(self, terrain_height):
        self.terrain.randomize(terrain_height)

    def set_filter_params(self, cutoff, order, robot_index=1):
        for i in range(p.getNumJoints(robot_index)):
            joint_info = p.getJointInfo(robot_index, i)
            name = joint_info[1].decode('utf-8')
            if name in ["LLB", "LLF", "LRF", "LRB", "RLB", "RLF", "RRF", "RRB"]:
                self.pressure_sensors[robot_index][name] = PressureSensor(name, i, robot_index, cutoff, order)

    def reset(self, robot_index=1):
        # set joints to initial position
        for name in self.joints[robot_index].keys():
            joint = self.joints[robot_index][name]
            try:
                pos_in_rad = math.radians(self.initial_joint_positions[name])
            except KeyError:
                pos_in_rad = 0
            joint.reset_position(pos_in_rad, 0)
            joint.set_position(pos_in_rad)

        # reset body pose and velocity
        p.resetBasePositionAndOrientation(robot_index, self.start_position, self.start_orientation)
        p.resetBaseVelocity(robot_index, [0, 0, 0], [0, 0, 0])

    def handle_gui(self):
        # get keyboard events if gui is active
        if self.gui:
            # check if simulation should continue currently
            while True:
                # reset if R-key was pressed
                rKey = ord('r')
                # gravity
                nKey = ord('n')
                # real time
                tKey = ord('t')
                # real time factor
                zKey = ord('z')
                # randomize terrain
                fKey = ord('f')
                # pause
                spaceKey = p.B3G_SPACE
                # rotate robot for testing
                jKey = ord('j')
                kKey = ord('k')
                lKey = ord('l')
                # move robot for testing
                qKey = ord('q')
                aKey = ord('a')
                sKey = ord('s')
                dKey = ord('d')
                keys = p.getKeyboardEvents()
                if rKey in keys and keys[rKey] & p.KEY_WAS_TRIGGERED:
                    self.reset()
                if spaceKey in keys and keys[spaceKey] & p.KEY_WAS_TRIGGERED:
                    self.paused = not self.paused
                if nKey in keys and keys[nKey] & p.KEY_WAS_TRIGGERED:
                    self.gravity = not self.gravity
                    self.set_gravity(self.gravity)  
                    print(f"gravity {self.gravity}")
                if tKey in keys and keys[tKey] & p.KEY_WAS_TRIGGERED:
                    self.realtime = not self.realtime
                    print("Realtime is " + str(self.realtime))
                if zKey in keys and keys[zKey] & p.KEY_WAS_TRIGGERED:
                    self.time_multiplier = (self.time_multiplier + 1) % 3
                    print(self.time_multiplier)
                if fKey in keys and keys[fKey] & p.KEY_WAS_TRIGGERED:
                    # generate new terrain
                    self.terrain.randomize(self.terrain_height)
                if jKey in keys and keys[jKey] & p.KEY_WAS_TRIGGERED:
                    pos, rpy = self.get_robot_pose_rpy()
                    self.reset_robot_pose_rpy(pos, (rpy[0] + math.radians(30), rpy[1], rpy[2]))
                if kKey in keys and keys[kKey] & p.KEY_WAS_TRIGGERED:
                    pos, rpy = self.get_robot_pose_rpy()
                    self.reset_robot_pose_rpy(pos, (rpy[0], rpy[1] + math.radians(30), rpy[2]))
                if lKey in keys and keys[lKey] & p.KEY_WAS_TRIGGERED:
                    pos, rpy = self.get_robot_pose_rpy()
                    self.reset_robot_pose_rpy(pos, (rpy[0], rpy[1], rpy[2] + math.radians(30)))
                if qKey in keys and keys[qKey] & p.KEY_WAS_TRIGGERED:
                    pos, quat = self.get_robot_pose()
                    self.reset_robot_pose((pos[0] + 0.1, pos[1], pos[2]), quat)
                if aKey in keys and keys[aKey] & p.KEY_WAS_TRIGGERED:
                    pos, quat = self.get_robot_pose()
                    self.reset_robot_pose((pos[0], pos[1] + 0.1, pos[2]), quat)
                if sKey in keys and keys[sKey] & p.KEY_WAS_TRIGGERED:
                    pos, quat = self.get_robot_pose()
                    self.reset_robot_pose((pos[0] - 0.1, pos[1], pos[2]), quat)
                if dKey in keys and keys[dKey] & p.KEY_WAS_TRIGGERED:
                    pos, quat = self.get_robot_pose()
                    self.reset_robot_pose((pos[0], pos[1] - 0.1, pos[2]), quat)
                if not self.paused:
                    break

            # sleep long enough to run the simulation in real time and not in accelerated speed
            if self.realtime:
                # wait till one timestep has actually passed in real time
                time_to_sleep = max(0, self.timestep - (time() - self.last_wall_time)) * (self.time_multiplier + 1)
                sleep(time_to_sleep)

        self.last_wall_time = time()
        self.time += self.timestep

    def step_pressure_filters(self, robot_index=1):
        for name, ps in self.pressure_sensors[robot_index].items():
            ps.filter_step()

    def step(self):
        self.handle_gui()
        p.stepSimulation()
        for index in self.robot_indexes:
            self.step_pressure_filters(index)

    def set_gravity(self, active):
        if active:
            p.setGravity(0, 0, -9.81)
        else:
            p.setGravity(0, 0, 0)

    def set_robot_pose(self, position, orientation, robot_index=1):
        p.resetBasePositionAndOrientation(robot_index, position, orientation)

    def reset_simulation(self):
        p.resetSimulation()
        self.load_models()

    def reset_robot_pose(self, position, orientation, reset_joints=False, robot_index=1):
        # reset body pose and velocity
        p.resetBasePositionAndOrientation(robot_index, position, orientation)
        p.resetBaseVelocity(robot_index, [0, 0, 0], [0, 0, 0])
        if reset_joints:
            # we need to reset all joints to, otherwise they still have velocity
            for name in self.joints[robot_index]:
                joint = self.joints[robot_index][name]
                try:
                    pos_in_rad = math.radians(self.initial_joint_positions[name])
                except KeyError:
                    pos_in_rad = 0
                joint.reset_position(pos_in_rad, 0)
                joint.set_position(pos_in_rad)

    def reset_robot_pose_rpy(self, position, rpy, robot_index=1):
        quat = tf_transformations.quaternion_from_euler(*rpy)
        self.reset_robot_pose(position, quat, robot_index=robot_index)

    def get_robot_pose(self, robot_index=1):
        (x, y, z), (qx, qy, qz, qw) = p.getBasePositionAndOrientation(robot_index)
        return (x, y, z), (qx, qy, qz, qw)

    def get_imu_quaternion(self):
        # imu orientation has roll and pitch relative to gravity vector. yaw in world frame
        _, robot_quat_in_world = self.get_robot_pose()
        # change order to transform3d standard
        robot_quat_in_world = (robot_quat_in_world[3], robot_quat_in_world[0], robot_quat_in_world[1], robot_quat_in_world[2])
        # get global yaw
        yrp_world_frame = quat2euler(robot_quat_in_world, axes='szxy')
        # remove global yaw rotation from roll and pitch
        yaw_quat = euler2quat(yrp_world_frame[0], 0, 0, axes='szxy')
        rp = rotate_vector((yrp_world_frame[1], yrp_world_frame[2], 0), qinverse(yaw_quat))
        # save in correct order
        rpy = [rp[0], rp[1], 0]
        # convert to quaternion
        quat_wxyz = euler2quat(*rpy)
        # change order to ros standard
        return quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]

    def get_robot_pose_rpy(self, robot_index=1):
        (x, y, z), (qx, qy, qz, qw) = p.getBasePositionAndOrientation(robot_index)
        (roll, pitch, yaw) = p.getEulerFromQuaternion((qx, qy, qz, qw))
        return (x, y, z), (roll, pitch, yaw)

    def get_link_pose(self, link_name, robot_index=1):
        return p.getLinkState(robot_index, self.links[robot_index][link_name])[0]

    def get_robot_velocity(self, robot_index=1):
        # these are in world coordinate frame
        (vx, vy, vz), (vr, vp, vy) = p.getBaseVelocity(robot_index)
        # rotate to robot frame
        _, (x, y, z, w) = self.get_robot_pose()
        # rotation matrix
        M = quat2mat((w, x, y, z))
        # velocities as vector
        v = np.array([vr, vp, vy]).T
        angular_vel_robot_frame = np.matmul(M.T, v)
        return (vx, vy, vz), angular_vel_robot_frame

    def get_joint_names(self, robot_index=1):
        names = []
        for name in self.joints[robot_index]:
            joint = self.joints[robot_index][name]
            names.append(joint.name)
        return names

    def get_joint_position(self, name, robot_index=1):
        return self.joints[robot_index][name].get_position()

    def get_joint_values(self, joint_names, scaled=False, robot_index=1):
        joint_positions = []
        joint_velocities = []
        joint_torques = []
        for joint_name in joint_names:
            joint = self.joints[robot_index][joint_name]
            pos, vel, tor = joint.update()
            if scaled:
                joint_positions.append(joint.get_scaled_position())
                joint_velocities.append(joint.get_scaled_velocity())
            else:
                joint_positions.append(pos)
                joint_velocities.append(vel)
            joint_torques.append(tor)
        return joint_positions, joint_velocities, joint_torques

    def get_link_values(self, link_name, robot_index=1):
        _, _, _, _, pos_in_world, quat_in_world, lin_vel_in_world, ang_vel_in_world = p.getLinkState(
            robot_index, self.links[robot_index][link_name], 1, 0)
        return pos_in_world, quat_in_world, lin_vel_in_world, ang_vel_in_world

    def get_base_position_and_orientation(self, robot_index=1):
        return p.getBasePositionAndOrientation(robot_index)

    def get_sensor_force(self, sensor_name, filtered, robot_index=1):
        force = self.pressure_sensors[robot_index][sensor_name].get_force()
        if filtered:
            if force[1] is None:
                return 0
            else:
                return force[1]
        else:
            if force[0] is None:
                return 0
            else:
                return force[0]

    def set_joint_position(self, joint_name, position, scaled=False, relative=False, robot_index=1):
        joint = self.joints[robot_index][joint_name]
        if scaled:
            joint.set_scaled_position(position, relative)
        else:
            joint.set_position(position, relative)

    def set_alpha(self, alpha, robot_index=1):
        ref_col = [1, 1, 1, alpha]
        p.changeVisualShape(robot_index, -1, rgbaColor=ref_col)
        for l in range(p.getNumJoints(robot_index)):
            p.changeVisualShape(robot_index, l, rgbaColor=ref_col)

    def reset_joint_to_position(self, joint_name, pos_in_rad, velocity=0, robot_index=1):
        joint = self.joints[robot_index][joint_name]
        joint.reset_position(pos_in_rad, velocity)
        joint.set_position(pos_in_rad)

    def reset_base_position_and_orientation(self, pos, quat, robot_index=1):
        p.resetBasePositionAndOrientation(robot_index, pos, quat)

    def reset_base_velocity(self, lin_vel, ang_vel, robot_index=1):
        p.resetBaseVelocity(robot_index, lin_vel, ang_vel)

    def reset_joints_to_init_pos(self, robot_index=1):
        for name in self.initial_joint_positions.keys():
            self.reset_joint_to_position(name, math.radians(self.initial_joint_positions[name]), velocity=0,
                                         robot_index=robot_index)

    def reset_pressure_filters(self, robot_index=1):
        for sensor in self.pressure_sensors[robot_index].values():
            sensor.reset()

    def apply_external_force_to_base(self, force, robot_index=1):
        p.applyExternalForce(robot_index, self.torso_ids[robot_index], force, [0, 0, 0], flags=p.WORLD_FRAME)

    def apply_external_torque_to_base(self, torque, robot_index=1):
        p.applyExternalTorque(robot_index, self.torso_ids[robot_index], torque, flags=p.WORLD_FRAME)

    def convert_radiant_to_scaled(self, joint_name, radiant, robot_index=1):
        return self.joints[robot_index][joint_name].convert_radiant_to_scaled(radiant)

    def convert_scaled_to_radiant(self, joint_name, scaled, robot_index=1):
        return self.joints[robot_index][joint_name].convert_scaled_to_radiant(scaled)


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
        self.mid_position = 0.5 * (self.lowerLimit + self.upperLimit)
        position, velocity, forces, applied_torque = p.getJointState(self.body_index,
                                                                     self.joint_index)
        self.state = position, velocity, forces, applied_torque

    def update(self):
        """
        Called just once per step to update state from simulation. Improves performance.
        """
        position, velocity, forces, applied_torque = p.getJointState(self.body_index,
                                                                     self.joint_index)
        self.state = position, velocity, forces, applied_torque
        return position, velocity, applied_torque

    def reset_position(self, position, velocity):
        p.resetJointState(self.body_index, self.joint_index, targetValue=position,
                          targetVelocity=velocity)

    def disable_motor(self):
        p.setJointMotorControl2(self.body_index, self.joint_index,
                                controlMode=p.POSITION_CONTROL, targetPosition=0,
                                targetVelocity=0, positionGain=0.1, velocityGain=0.1, force=0)

    def set_position(self, position, relative=False):
        # enforce limits
        if relative:
            position = position + self.state[0]
        position = min(self.upperLimit, max(self.lowerLimit, position))
        p.setJointMotorControl2(self.body_index, self.joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=position, force=self.max_force,
                                maxVelocity=self.max_velocity)

    def set_scaled_position(self, position, relative):
        self.set_position(self.convert_scaled_to_radiant(position), relative=relative)

    def reset_scaled_position(self, position):
        # sets position inside limits with a given position values in [-1, 1]
        self.reset_position(self.convert_scaled_to_radiant(position), 0)

    def get_state(self):
        return self.state

    def get_position(self):
        return self.state[0]

    def get_scaled_position(self):
        return self.convert_radiant_to_scaled(self.state[0])

    def get_velocity(self):
        return self.state[1]

    def get_scaled_velocity(self):
        return self.get_velocity() * 0.01

    def get_torque(self):
        position, velocity, forces, applied_torque = self.state
        return applied_torque

    def convert_radiant_to_scaled(self, pos):
        # helper method to convert to scaled position between [-1,1] for this joint using min max scaling
        return 2 * (pos - self.mid_position) / (self.upperLimit - self.lowerLimit)

    def convert_scaled_to_radiant(self, position):
        # helper method to convert to scaled position for this joint using min max scaling
        return position * (self.upperLimit - self.lowerLimit) / 2 + self.mid_position


class PressureSensor:
    def __init__(self, name, joint_index, body_index, cutoff, order):
        self.joint_index = joint_index
        self.name = name
        self.body_index = body_index
        nyq = 240 * 0.5  # nyquist frequency from simulation frequency
        normalized_cutoff = cutoff / nyq  # cutoff freq in hz
        self.filter_b, self.filter_a = signal.butter(order, normalized_cutoff, btype='low')
        self.filter_state = None
        self.reset()
        self.unfiltered = 0
        self.filtered = [0]

    def reset(self):
        self.filter_state = signal.lfilter_zi(self.filter_b, self.filter_a)

    def filter_step(self, unfiltered=None):
        if unfiltered is None:
            self.unfiltered = p.getJointState(self.body_index, self.joint_index)[2][2] * -1
        self.filtered, self.filter_state = signal.lfilter(self.filter_b, self.filter_a, [self.unfiltered],
                                                          zi=self.filter_state)

    def get_force(self):
        return max(self.unfiltered, 0), max(self.filtered[0], 0)

    def get_value(self, type):
        if type == "filtered":
            return max(self.filtered[0], 0)
        elif type == "raw":
            return max(self.unfiltered, 0)
        elif type == "binary":
            if self.unfiltered > 10:
                return 1
            else:
                return 0
        else:
            print(f"type '{type}' not know")
