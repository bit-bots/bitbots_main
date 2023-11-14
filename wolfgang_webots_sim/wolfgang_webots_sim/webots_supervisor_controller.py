from controller import Supervisor, Keyboard, Node

from rclpy.node import Node as RclpyNode
from geometry_msgs.msg import Quaternion, Pose, Point, Twist
from gazebo_msgs.msg import ModelStates
from bitbots_msgs.srv import SetObjectPose, SetObjectPosition
from rclpy.time import Time

from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty

import transforms3d
import numpy as np

G = 9.81


class SupervisorController:
    def __init__(self, ros_node: Node = None, ros_active=False, mode='normal', base_ns='/', model_states_active=True, robot="wolfgang"):
        """
        The SupervisorController, a Webots controller that can control the world.
        Set the environment variable WEBOTS_ROBOT_NAME to "supervisor_robot" if used with 1_bot.wbt or 4_bots.wbt.

        :param ros_active: Whether to publish ROS messages
        :param mode: Webots mode, one of 'normal', 'paused', or 'fast'
        :param base_ns: The namespace of this node, can normally be left empty
        """
        self.ros_node = ros_node
        if self.ros_node is None:
            self.ros_node = RclpyNode('supervisor_controller')
        # requires WEBOTS_ROBOT_NAME to be set to "supervisor_robot"
        self.ros_active = ros_active
        self.model_states_active = model_states_active
        self.time = 0
        self.clock_msg = Clock()
        self.supervisor = Supervisor()
        self.keyboard_activated = False

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

        self.robot_nodes = {}
        self.translation_fields = {}
        self.rotation_fields = {}
        self.joint_nodes = {}
        self.link_nodes = {}

        # set reset height based on the robot, so that we can reset different robots easily
        self.robot_type = robot
        reset_heights = {"wolfgang": 0.42, "robotis_op2": 0.35, "op3": 0.35, "rfc": 0.40, "chape": 0.30,
                         "mrl_hsl": 0.40, "nugus": 0.5, "bez": 0.30}
        if not self.robot_type in reset_heights.keys():
            self.ros_node.get_logger().warn(f"Robot type {self.robot_type} has no reset height defined. Will use 1m")
            self.reset_height = 1
        else:
            self.reset_height = reset_heights[self.robot_type]

        root = self.supervisor.getRoot()
        children_field = root.getField('children')
        children_count = children_field.getCount()
        for i in range(children_count):
            node = children_field.getMFNode(i)
            name_field = node.getField('name')
            if name_field is not None and node.getType() == Node.ROBOT:
                # this is a robot
                name = name_field.getSFString()
                if name == "supervisor_robot":
                    continue
                self.robot_nodes[name] = node
                self.translation_fields[name] = node.getField("translation")
                self.rotation_fields[name] = node.getField("rotation")
                self.joint_nodes[name], self.link_nodes[name] = self.collect_joint_and_link_node_references(node, {},
                                                                                                            {})
        if self.ros_active:
            # need to handle these topics differently or we will end up having a double //
            if base_ns == "":
                clock_topic = "/clock"
                model_topic = "/model_states"
            else:
                clock_topic = base_ns + "clock"
                model_topic = base_ns + "model_states"
            self.clock_publisher = self.ros_node.create_publisher(Clock, clock_topic, 1)
            self.model_state_publisher = self.ros_node.create_publisher(ModelStates, model_topic, 1)
            self.reset_service = self.ros_node.create_service(Empty, base_ns + "reset", self.reset)
            self.reset_pose_service = self.ros_node.create_service(Empty, base_ns + "reset_pose", self.set_initial_poses)
            self.set_robot_pose_service = self.ros_node.create_service(SetObjectPose, base_ns + "set_robot_pose",
                                                                   self.robot_pose_callback)
            self.reset_ball_service = self.ros_node.create_service(Empty, base_ns + "reset_ball", self.reset_ball)
            self.set_ball_position_service = self.ros_node.create_service(SetObjectPosition, base_ns + "set_ball_position",
                                                                      self.ball_pos_callback)

        self.world_info = self.supervisor.getFromDef("world_info")
        self.ball = self.supervisor.getFromDef("ball")

    def collect_joint_and_link_node_references(self, node, joint_dict, link_dict):
        # this is a recursive function that iterates through the whole robot as this seems to be the only way to
        # get all joints
        # add node if it is a joint
        if node.getType() == Node.SOLID:
            name = node.getDef()
            if name == "":
                self.ros_node.get_logger().warn("Proto has link without name", once=True)
            link_dict[name] = node
        if node.getType() in [Node.HINGE_JOINT, Node.HINGE_2_JOINT]:
            name = node.getDef()
            if name == "":
                self.ros_node.get_logger().warn("Proto has joint without name", once=True)
            # substract the "Joint" keyword due to naming convention
            if name[-5:] != "Joint":
                self.ros_node.get_logger().warn(f"Joint names are expected to end with \"Joint\". \"{name}\" does not.", once=True)
            name = name[:-5]
            joint_dict[name] = node
            # the joints dont have children but an "endpoint" that we need to search through
            if node.isProto():
                endpoint_field = node.getProtoField('endPoint')
            else:
                endpoint_field = node.getField('endPoint')
            endpoint_node = endpoint_field.getSFNode()
            self.collect_joint_and_link_node_references(endpoint_node, joint_dict, link_dict)
        # needs to be done because Webots has two different getField functions for proto nodes and normal nodes
        if node.isProto():
            children_field = node.getProtoField('children')
        else:
            children_field = node.getField('children')
        if children_field is not None:
            for i in range(children_field.getCount()):
                child = children_field.getMFNode(i)
                self.collect_joint_and_link_node_references(child, joint_dict, link_dict)
        return joint_dict, link_dict

    def step_sim(self):
        self.time += self.timestep / 1000.0
        self.supervisor.step(self.timestep)

    def step(self):
        self.step_sim()
        if self.ros_active:
            self.publish_clock()
            if self.model_states_active:
                self.publish_model_states()

    def handle_gui(self):
        if not self.keyboard_activated:
            self.keyboard = Keyboard()
            self.keyboard.enable(100)
            self.keyboard_activated = True
        key = self.keyboard.getKey()
        if key == ord('R'):
            self.reset()
        elif key == ord('P'):
            self.set_initial_poses()
        elif key == Keyboard.SHIFT + ord('R'):
            try:
                self.reset_ball()
            except AttributeError:
                print("No ball in simulation that can be reset")
        return key

    def publish_clock(self):
        self.clock_msg.clock = Time(seconds=int(self.time), nanoseconds=self.time % 1 * 1e9).to_msg()
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

    def reset(self, request=None, response=Empty.Response()):
        self.supervisor.simulationReset()
        self.supervisor.simulationResetPhysics()
        return response

    def reset_robot_init(self, name="amy"):
        self.robot_nodes[name].loadState('__init__')
        self.robot_nodes[name].resetPhysics()

    def set_initial_poses(self, request=None, response=Empty.Response()):
        self.reset_robot_pose_rpy([-1, 3, self.reset_height], [0, 0.24, -1.57], name="amy")
        self.reset_robot_pose_rpy([-1, -3, self.reset_height], [0, 0.24, 1.57], name="rory")
        self.reset_robot_pose_rpy([-3, 3, self.reset_height], [0, 0.24, -1.57], name="jack")
        self.reset_robot_pose_rpy([-3, -3, self.reset_height], [0, 0.24, 1.57], name="donna")
        self.reset_robot_pose_rpy([0, 6, self.reset_height], [0, 0.24, -1.57], name="melody")
        return response

    def robot_pose_callback(self, request=None, response=SetObjectPose.Response()):
        self.reset_robot_pose([request.pose.position.x, request.pose.position.y, request.pose.position.z],
                              [request.pose.orientation.x, request.pose.orientation.y, request.pose.orientation.z,
                               request.pose.orientation.w], request.object_name)
        return response

    def reset_ball(self, request=None, response=Empty.Response()):
        self.ball.getField("translation").setSFVec3f([0, 0, 0.0772])
        self.ball.getField("rotation").setSFRotation([0, 0, 1, 0])
        self.ball.resetPhysics()
        return response

    def ball_pos_callback(self, request=None, response=SetObjectPosition.Response()):
        self.set_ball_pose([request.position.x, request.position.y, request.position.z])
        return response

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
            self.ros_node.get_logger().warn(f"Could not find link \"{link}\"")
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
        if self.model_state_publisher.get_subscription_count() != 0:
            msg = ModelStates()
            for robot_name, robot_node in self.robot_nodes.items():
                position, orientation = self.get_robot_pose_quat(name=robot_name)
                robot_pose = Pose()
                robot_pose.position = Point(x=position[0], y=position[1], z=position[2])
                robot_pose.orientation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2],
                                                    w=orientation[3])
                msg.name.append(robot_name)
                msg.pose.append(robot_pose)
                lin_vel, ang_vel = self.get_robot_velocity(robot_name)
                twist = Twist()
                twist.linear.x = lin_vel[0]
                twist.linear.y = lin_vel[1]
                twist.linear.z = lin_vel[2]
                twist.angular.x = ang_vel[0]
                twist.angular.y = ang_vel[1]
                twist.angular.z = ang_vel[2]
                msg.twist.append(twist)

                head_node = robot_node.getFromProtoDef("head")
                head_position = head_node.getPosition()
                head_orientation = head_node.getOrientation()
                head_orientation_quat = transforms3d.quaternions.mat2quat(np.reshape(head_orientation, (3, 3)))
                head_pose = Pose()
                head_pose.position = Point(x=head_position[0], y=head_position[1], z=head_position[2])
                head_pose.orientation = Quaternion(x=head_orientation_quat[1], y=head_orientation_quat[2],
                                                   z=head_orientation_quat[3], w=head_orientation_quat[0])
                msg.name.append(robot_name + "_head")
                msg.pose.append(head_pose)

            if self.ball is not None:
                ball_position = self.ball.getField("translation").getSFVec3f()
                ball_pose = Pose()
                ball_pose.position = Point(x=ball_position[0], y=ball_position[1], z=ball_position[2])
                ball_pose.orientation = Quaternion()
                msg.name.append("ball")
                msg.pose.append(ball_pose)

            self.model_state_publisher.publish(msg)
