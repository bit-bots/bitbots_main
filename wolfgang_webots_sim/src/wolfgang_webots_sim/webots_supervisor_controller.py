from controller import Supervisor

import rospy
from geometry_msgs.msg import Quaternion, Pose, Point
from gazebo_msgs.msg import ModelStates
from bitbots_msgs.srv import SetObjectPose, SetObjectPoseResponse, SetObjectPosition, SetObjectPositionResponse

from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty, EmptyResponse

import transforms3d
import numpy as np

G = 9.81


class SupervisorController:
    def __init__(self, ros_active=False, mode='normal', do_ros_init=True, base_ns='', model_states_active=True):
        """
        The SupervisorController, a Webots controller that can control the world.
        Set the environment variable WEBOTS_ROBOT_NAME to "supervisor_robot" if used with 1_bot.wbt or 4_bots.wbt.

        :param ros_active: Whether to publish ROS messages
        :param mode: Webots mode, one of 'normal', 'paused', or 'fast'
        :param do_ros_init: Whether rospy.init_node should be called
        :param base_ns: The namespace of this node, can normally be left empty
        """
        # requires WEBOTS_ROBOT_NAME to be set to "supervisor_robot"
        self.ros_active = ros_active
        self.model_states_active = model_states_active
        self.time = 0
        self.clock_msg = Clock()

        self.supervisor = Supervisor()

        if mode == 'normal':
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
        elif mode == 'paused':
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
        elif mode == 'fast':
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)
        else:
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)

        self.motors = []
        self.sensors = []
        self.timestep = int(self.supervisor.getBasicTimeStep())

        # resolve the node for corresponding name
        self.robot_names = ["amy", "rory", "jack", "donna"]
        self.robot_nodes = {}
        self.translation_fields = {}
        self.rotation_fields = {}

        # check if None
        for name in self.robot_names:
            node = self.supervisor.getFromDef(name)
            if node is not None:
                self.robot_nodes[name] = node
                self.translation_fields[name] = node.getField("translation")
                self.rotation_fields[name] = node.getField("rotation")

        if self.ros_active:
            # need to handle these topics differently or we will end up having a double //
            if base_ns == "":
                clock_topic = "/clock"
                model_topic = "/model_states"
            else:
                clock_topic = base_ns + "clock"
                model_topic = base_ns + "model_states"
            if do_ros_init:
                rospy.init_node("webots_ros_supervisor", argv=['clock:=' + clock_topic])
            self.clock_publisher = rospy.Publisher(clock_topic, Clock, queue_size=1)
            self.model_state_publisher = rospy.Publisher(model_topic, ModelStates, queue_size=1)
            self.reset_service = rospy.Service(base_ns + "reset", Empty, self.reset)
            self.reset_pose_service = rospy.Service(base_ns + "reset_pose", Empty, self.set_initial_poses)
            self.set_robot_pose_service = rospy.Service(base_ns + "set_robot_pose", SetObjectPose,
                                                        self.robot_pose_callback)
            self.reset_ball_service = rospy.Service(base_ns + "reset_ball", Empty, self.reset_ball)
            self.set_ball_position_service = rospy.Service(base_ns + "set_ball_position", SetObjectPosition,
                                                           self.ball_pos_callback)

        self.world_info = self.supervisor.getFromDef("world_info")
        self.ball = self.supervisor.getFromDef("ball")

    def step_sim(self):
        self.time += self.timestep / 1000
        self.supervisor.step(self.timestep)

    def step(self):
        self.step_sim()
        if self.ros_active:
            self.publish_clock()
            if self.model_states_active:
                self.publish_model_states()

    def publish_clock(self):
        self.clock_msg.clock = rospy.Time.from_seconds(self.time)
        self.clock_publisher.publish(self.clock_msg)

    def set_gravity(self, active):
        if active:
            self.world_info.getField("gravity").setSFFloat(9.81)
        else:
            self.world_info.getField("gravity").setSFFloat(0)

    def set_self_collision(self, active, name="amy"):
        self.robot_nodes[name].getField("selfCollision").setSFBool(active)

    def reset_robot_pose(self, pos, quat, name="amy"):
        self.set_robot_pose_quat(pos, quat, name)
        if name in self.robot_nodes:
            self.robot_nodes[name].resetPhysics()

    def reset_robot_pose_rpy(self, pos, rpy, name="amy"):
        self.set_robot_pose_rpy(pos, rpy, name)
        if name in self.robot_nodes:
            self.robot_nodes[name].resetPhysics()

    def reset(self, req=None):
        self.supervisor.simulationReset()
        self.supervisor.simulationResetPhysics()
        return EmptyResponse()

    def reset_robot_init(self, name="amy"):
        self.robot_nodes[name].loadState('__init__')
        self.robot_nodes[name].resetPhysics()

    def set_initial_poses(self, req=None):
        self.reset_robot_pose_rpy([-1, 3, 0.42], [0, 0.24, -1.57], name="amy")
        self.reset_robot_pose_rpy([-1, -3, 0.42], [0, 0.24, 1.57], name="rory")
        self.reset_robot_pose_rpy([-3, 3, 0.42], [0, 0.24, -1.57], name="jack")
        self.reset_robot_pose_rpy([-3, -3, 0.42], [0, 0.24, 1.57], name="donna")
        self.reset_robot_pose_rpy([0, 6, 0.42], [0, 0.24, -1.57], name="melody")
        return EmptyResponse()

    def robot_pose_callback(self, req=None):
        self.reset_robot_pose([req.pose.position.x, req.pose.position.y, req.pose.position.z],
                              [req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z,
                               req.pose.orientation.w], req.object_name)
        return SetObjectPoseResponse()

    def reset_ball(self, req=None):
        self.ball.getField("translation").setSFVec3f([0, 0, 0.0772])
        self.ball.getField("rotation").setSFRotation([0, 0, 1, 0])
        self.ball.resetPhysics()
        return EmptyResponse()

    def ball_pos_callback(self, req=None):
        self.set_ball_pose([req.position.x, req.position.y, req.position.z])
        return SetObjectPositionResponse()

    def set_ball_pose(self, pos):
        self.ball.getField("translation").setSFVec3f(list(pos))
        self.ball.resetPhysics()

    def get_ball_pose(self):
        return self.ball.getField("translation").getSFVec3f()

    def get_ball_velocity(self):
        if self.ball:
            return self.ball.getVelocity()

    def node(self):
        s = self.supervisor.getSelected()
        if s is not None:
            print(f"id: {s.getId()}, type: {s.getType()}, def: {s.getDef()}")

    def set_robot_axis_angle(self, axis, angle, name="amy"):
        if name in self.rotation_fields:
            self.rotation_fields[name].setSFRotation(list(np.append(axis, angle)))

    def set_robot_rpy(self, rpy, name="amy"):
        axis, angle = transforms3d.euler.euler2axangle(rpy[0], rpy[1], rpy[2], axes='sxyz')
        self.set_robot_axis_angle(axis, angle, name)

    def set_robot_quat(self, quat, name="amy"):
        axis, angle = transforms3d.quaternions.quat2axangle([quat[3], quat[0], quat[1], quat[2]])
        self.set_robot_axis_angle(axis, angle, name)

    def set_robot_position(self, pos, name="amy"):
        if name in self.translation_fields:
            self.translation_fields[name].setSFVec3f(list(pos))

    def set_robot_pose_rpy(self, pos, rpy, name="amy"):
        self.set_robot_position(pos, name)
        self.set_robot_rpy(rpy, name)

    def set_robot_pose_quat(self, pos, quat, name="amy"):
        self.set_robot_position(pos, name)
        self.set_robot_quat(quat, name)

    def get_robot_position(self, name="amy"):
        if name in self.translation_fields:
            return self.translation_fields[name].getSFVec3f()

    def get_robot_orientation_axangles(self, name="amy"):
        if name in self.rotation_fields:
            return self.rotation_fields[name].getSFRotation()

    def get_robot_orientation_rpy(self, name="amy"):
        ax_angle = self.get_robot_orientation_axangles(name)
        return list(transforms3d.euler.axangle2euler(ax_angle[:3], ax_angle[3], axes='sxyz'))

    def get_robot_orientation_quat(self, name="amy"):
        ax_angle = self.get_robot_orientation_axangles(name)
        # transforms 3d uses scalar (i.e. the w part in the quaternion) first notation of quaternions, ros uses scalar last
        quat_scalar_first = transforms3d.quaternions.axangle2quat(ax_angle[:3], ax_angle[3])
        quat_scalar_last = np.append(quat_scalar_first[1:], quat_scalar_first[0])
        return list(quat_scalar_last)

    def get_robot_pose_rpy(self, name="amy"):
        return self.get_robot_position(name), self.get_robot_orientation_rpy(name)

    def get_robot_pose_quat(self, name="amy"):
        return self.get_robot_position(name), self.get_robot_orientation_quat(name)

    def get_robot_velocity(self, name="amy"):
        velocity = self.robot_nodes[name].getVelocity()
        return velocity[:3], velocity[3:]

    def get_link_pose(self, link, name="amy"):
        link_node = self.robot_nodes[name].getFromProtoDef(link)
        if not link_node:
            return None
        link_position = link_node.getPosition()
        mat = link_node.getOrientation()
        link_orientation = transforms3d.quaternions.mat2quat(np.array(mat))
        return link_position, np.append(link_orientation[1:], link_orientation[0])

    def get_link_velocities(self, link, name="amy"):
        """Returns linear and angular velocities"""
        link_node = self.robot_nodes[name].getFromProtoDef(link)
        velocity = link_node.getVelocity()
        return velocity[:3], velocity[3:]

    def publish_model_states(self):
        if self.model_state_publisher.get_num_connections() != 0:
            msg = ModelStates()
            for robot_name, robot_node in self.robot_nodes.items():
                position, orientation = self.get_robot_pose_quat(name=robot_name)
                robot_pose = Pose()
                robot_pose.position = Point(*position)
                robot_pose.orientation = Quaternion(*orientation)
                msg.name.append(robot_name)
                msg.pose.append(robot_pose)

                head_node = robot_node.getFromProtoDef("head")
                head_position = head_node.getPosition()
                head_orientation = head_node.getOrientation()
                head_orientation_quat = transforms3d.quaternions.mat2quat(np.reshape(head_orientation, (3, 3)))
                head_pose = Pose()
                head_pose.position = Point(*head_position)
                head_pose.orientation = Quaternion(head_orientation_quat[1], head_orientation_quat[2],
                                                   head_orientation_quat[3], head_orientation_quat[0])
                msg.name.append(robot_name + "_head")
                msg.pose.append(head_pose)

            ball_position = self.ball.getField("translation").getSFVec3f()
            ball_pose = Pose()
            ball_pose.position = Point(*ball_position)
            ball_pose.orientation = Quaternion()
            msg.name.append("ball")
            msg.pose.append(ball_pose)
            self.model_state_publisher.publish(msg)
