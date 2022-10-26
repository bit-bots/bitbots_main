import time
import rclpy
from rclpy.node import Node
from bitbots_msgs.msg import FootPressure, JointCommand
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32, Bool
from tf_transformations import euler_from_quaternion
from bitbots_utils.transforms import xyzw2wxyz
from transforms3d.quaternions import rotate_vector, qinverse

class ROSInterface():
    def __init__(self, node: Node, simulation, namespace='', declare_parameters=True):
        self.node = node
        self.namespace = namespace
        self.simulation = simulation
        self.namespace = namespace
        self.last_time = time.time()
        self.last_linear_vel = (0, 0, 0)

        # messages
        self.real_time_msg = Float32()
        self.joint_state_msg = JointState()
        self.joint_state_msg.header.frame_id = "base_link"
        self.joint_state_msg.name = self.simulation.get_joint_names()
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_frame"
        self.clock_msg = Clock()
        self.foot_msg_left = FootPressure()
        self.foot_msg_left.header.frame_id = 'l_sole'
        self.foot_msg_right = FootPressure()
        self.foot_msg_right.header.frame_id = 'r_sole'
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        # we just use the first robot
        self.robot_index = self.simulation.robot_indexes[0]

        # necessary to use this as part of a different node
        if declare_parameters:
            self.node.declare_parameter("contact_stiffness", 0.0)
            self.node.declare_parameter("spinning_friction", 0.0)
            self.node.declare_parameter("contact_damping", 0.0)
            self.node.declare_parameter("lateral_friction", 0.0)
            self.node.declare_parameter("rolling_friction", 0.0)
            self.node.declare_parameter("restitution", 0.0)
            self.node.declare_parameter("cutoff", 0)
            self.node.declare_parameter("order", 0)

        self.node.add_on_set_parameters_callback(self.parameters_callback)

        self.simulation.set_dynamics(
            contact_damping=self.node.get_parameter("contact_damping").get_parameter_value().double_value,
            contact_stiffness=self.node.get_parameter("contact_stiffness").get_parameter_value().double_value,
            lateral_friction=self.node.get_parameter("lateral_friction").get_parameter_value().double_value,
            spinning_friction=self.node.get_parameter("spinning_friction").get_parameter_value().double_value,
            rolling_friction=self.node.get_parameter("rolling_friction").get_parameter_value().double_value,
            restitution=self.node.get_parameter("restitution").get_parameter_value().double_value)
        self.simulation.set_filter_params(self.node.get_parameter("cutoff").get_parameter_value().integer_value,
                                          self.node.get_parameter("order").get_parameter_value().integer_value)

        # publisher
        self.left_foot_pressure_publisher = self.node.create_publisher(FootPressure,
                                                                       self.namespace + "foot_pressure_left/raw", 1)
        self.right_foot_pressure_publisher = self.node.create_publisher(FootPressure,
                                                                        self.namespace + "foot_pressure_right/raw", 1)
        self.left_foot_pressure_publisher_filtered = self.node.create_publisher(FootPressure,
                                                                                self.namespace + "foot_pressure_left/filtered",
                                                                                1)
        self.right_foot_pressure_publisher_filtered = self.node.create_publisher(FootPressure,
                                                                                 self.namespace + "foot_pressure_right/filtered",
                                                                                 1)
        self.joint_publisher = self.node.create_publisher(JointState, self.namespace + "joint_states", 1)
        self.imu_publisher = self.node.create_publisher(Imu, self.namespace + "imu/data_raw", 1)
        self.clock_publisher = self.node.create_publisher(Clock, self.namespace + "clock", 1)
        self.real_time_factor_publisher = self.node.create_publisher(Float32, self.namespace + "real_time_factor", 1)
        self.true_odom_publisher = self.node.create_publisher(Odometry, self.namespace + "true_odom", 1)
        self.cop_l_pub_ = self.node.create_publisher(PointStamped, self.namespace + "cop_l", 1)
        self.cop_r_pub_ = self.node.create_publisher(PointStamped, self.namespace + "cop_r", 1)

        # subscriber
        self.joint_goal_subscriber = self.node.create_subscription(JointCommand,
                                                                   self.namespace + "DynamixelController/command",
                                                                   self.joint_goal_cb, 1)

        self.reset_subscriber = self.node.create_subscription(Bool, self.namespace + "reset",
                                                              self.reset_cb, 1)

    def step(self):
        self.simulation.step()
        self.publish_joints()
        self.publish_imu()
        self.publish_foot_pressure()
        self.publish_true_odom()
        self.clock_msg.clock = Time(seconds=int(self.simulation.time),
                                    nanoseconds=self.simulation.time % 1 * 1e9).to_msg()
        self.clock_publisher.publish(self.clock_msg)
        self.compute_real_time_factor()

    def run_simulation(self, duration=None, sleep=0):
        start_time = self.node.get_clock().now().seconds_nanoseconds()[0] + \
                     self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9
        while rclpy.ok() and (duration is None or (self.node.get_clock().now().seconds_nanoseconds()[0] +
                                                   self.node.get_clock().now().seconds_nanoseconds()[1] / 1e9)
                              - start_time < duration):
            self.step()
            time.sleep(sleep)

    def compute_real_time_factor(self):
        time_now = time.time()
        self.real_time_msg.data = self.simulation.timestep / (time_now - self.last_time)
        self.last_time = time_now
        self.real_time_factor_publisher.publish(self.real_time_msg)

    def get_joint_state_msg(self):
        positions = []
        velocities = []
        efforts = []
        for name in self.joint_state_msg.name:
            joint = self.simulation.joints[self.robot_index][name]
            position, velocity, forces, applied_torque = joint.get_state()
            positions.append(position)
            velocities.append(velocity)
            efforts.append(applied_torque)
        self.joint_state_msg.position = positions
        self.joint_state_msg.velocity = velocities
        self.joint_state_msg.effort = efforts
        self.joint_state_msg.header.stamp = Time(seconds=int(self.simulation.time),
                                                 nanoseconds=self.simulation.time % 1 * 1e9).to_msg()
        return self.joint_state_msg

    def publish_joints(self):
        self.joint_publisher.publish(self.get_joint_state_msg())

    def get_imu_msg(self):
        position, orientation = self.simulation.get_robot_pose()
        self.imu_msg.orientation.x = orientation[0]
        self.imu_msg.orientation.y = orientation[1]
        self.imu_msg.orientation.z = orientation[2]
        self.imu_msg.orientation.w = orientation[3]
        linear_vel, angular_vel = self.simulation.get_robot_velocity()
        self.imu_msg.angular_velocity.x = angular_vel[0]
        self.imu_msg.angular_velocity.y = angular_vel[1]
        self.imu_msg.angular_velocity.z = angular_vel[2]
        # simple acceleration computation by using diff of velocities
        linear_acc = tuple(map(lambda i, j: i - j, self.last_linear_vel, linear_vel))
        self.last_linear_vel = linear_vel
        # adding gravity to the acceleration
        gravity_vector = (0, 0, 9.81)
        gravity_rotated = rotate_vector(gravity_vector, qinverse(xyzw2wxyz(orientation)))
        linear_acc = tuple([linear_acc[0] + gravity_rotated[0], linear_acc[1] + gravity_rotated[1], linear_acc[2] + gravity_rotated[2]])
        self.imu_msg.linear_acceleration.x = linear_acc[0]
        self.imu_msg.linear_acceleration.y = linear_acc[1]
        self.imu_msg.linear_acceleration.z = linear_acc[2]
        self.imu_msg.header.stamp = Time(seconds=int(self.simulation.time),
                                         nanoseconds=self.simulation.time % 1 * 1e9).to_msg()
        return self.imu_msg

    def publish_imu(self):
        self.imu_publisher.publish(self.get_imu_msg())

    def get_pressure_filtered_left(self):
        if len(self.simulation.pressure_sensors) == 0:
            self.node.get_logger().warn_once("No pressure sensors found in simulation model")
            return self.foot_msg_left
        f_llb = self.simulation.pressure_sensors[self.robot_index]["LLB"].get_force()
        f_llf = self.simulation.pressure_sensors[self.robot_index]["LLF"].get_force()
        f_lrf = self.simulation.pressure_sensors[self.robot_index]["LRF"].get_force()
        f_lrb = self.simulation.pressure_sensors[self.robot_index]["LRB"].get_force()
        self.foot_msg_left.left_back = float(f_llb[1])
        self.foot_msg_left.left_front = float(f_llf[1])
        self.foot_msg_left.right_front = float(f_lrf[1])
        self.foot_msg_left.right_back = float(f_lrb[1])
        return self.foot_msg_left

    def get_pressure_filtered_right(self):
        if len(self.simulation.pressure_sensors) == 0:
            self.node.get_logger().warn_once("No pressure sensors found in simulation model")
            return self.foot_msg_right
        f_rlb = self.simulation.pressure_sensors[self.robot_index]["RLB"].get_force()
        f_rlf = self.simulation.pressure_sensors[self.robot_index]["RLF"].get_force()
        f_rrf = self.simulation.pressure_sensors[self.robot_index]["RRF"].get_force()
        f_rrb = self.simulation.pressure_sensors[self.robot_index]["RRB"].get_force()
        self.foot_msg_right.left_back = float(f_rlb[1])
        self.foot_msg_right.left_front = float(f_rlf[1])
        self.foot_msg_right.right_front = float(f_rrf[1])
        self.foot_msg_right.right_back = float(f_rrb[1])
        return self.foot_msg_right

    def publish_foot_pressure(self):
        # some models dont have sensors
        if len(self.simulation.pressure_sensors) == 0:
            self.node.get_logger().warn_once("No pressure sensors found in simulation model")
            return

        f_llb = self.simulation.pressure_sensors[self.robot_index]["LLB"].get_force()
        f_llf = self.simulation.pressure_sensors[self.robot_index]["LLF"].get_force()
        f_lrf = self.simulation.pressure_sensors[self.robot_index]["LRF"].get_force()
        f_lrb = self.simulation.pressure_sensors[self.robot_index]["LRB"].get_force()

        f_rlb = self.simulation.pressure_sensors[self.robot_index]["RLB"].get_force()
        f_rlf = self.simulation.pressure_sensors[self.robot_index]["RLF"].get_force()
        f_rrf = self.simulation.pressure_sensors[self.robot_index]["RRF"].get_force()
        f_rrb = self.simulation.pressure_sensors[self.robot_index]["RRB"].get_force()

        self.foot_msg_left.left_back = float(f_llb[0])
        self.foot_msg_left.left_front = float(f_llf[0])
        self.foot_msg_left.right_front = float(f_lrf[0])
        self.foot_msg_left.right_back = float(f_lrb[0])
        self.left_foot_pressure_publisher.publish(self.foot_msg_left)

        self.foot_msg_right.left_back = float(f_rlb[0])
        self.foot_msg_right.left_front = float(f_rlf[0])
        self.foot_msg_right.right_front = float(f_rrf[0])
        self.foot_msg_right.right_back = float(f_rrb[0])
        self.right_foot_pressure_publisher.publish(self.foot_msg_right)

        self.foot_msg_left.left_back = float(f_llb[1])
        self.foot_msg_left.left_front = float(f_llf[1])
        self.foot_msg_left.right_front = float(f_lrf[1])
        self.foot_msg_left.right_back = float(f_lrb[1])
        self.left_foot_pressure_publisher_filtered.publish(self.foot_msg_left)

        self.foot_msg_right.left_back = float(f_rlb[1])
        self.foot_msg_right.left_front = float(f_rlf[1])
        self.foot_msg_right.right_front = float(f_rrf[1])
        self.foot_msg_right.right_back = float(f_rrb[1])
        self.right_foot_pressure_publisher_filtered.publish(self.foot_msg_right)

        # center position on foot
        pos_x = 0.085
        pos_y = 0.045
        threshold = 0.0

        cop_l = PointStamped()
        cop_l.header.frame_id = "l_sole"
        cop_l.header.stamp = Time(seconds=int(self.simulation.time),
                                  nanoseconds=self.simulation.time % 1 * 1e9).to_msg()
        sum_of_forces = f_llb[1] + f_llf[1] + f_lrf[1] + f_lrb[1]
        if sum_of_forces > threshold:
            cop_l.point.x = (f_llf[1] + f_lrf[1] - f_llb[1] - f_lrb[1]) * pos_x / sum_of_forces
            cop_l.point.x = max(min(cop_l.point.x, pos_x), -pos_x)
            cop_l.point.y = (f_llf[1] + f_llb[1] - f_lrf[1] - f_lrb[1]) * pos_y / sum_of_forces
            cop_l.point.y = max(min(cop_l.point.y, pos_y), -pos_y)
        else:
            cop_l.point.x = 0.0
            cop_l.point.y = 0.0
        self.cop_l_pub_.publish(cop_l)

        cop_r = PointStamped()
        cop_r.header.frame_id = "r_sole"
        cop_r.header.stamp = Time(seconds=int(self.simulation.time),
                                  nanoseconds=self.simulation.time % 1 * 1e9).to_msg()
        sum_of_forces = f_rlb[1] + f_rlf[1] + f_rrf[1] + f_rrb[1]
        if sum_of_forces > threshold:
            cop_r.point.x = (f_rlf[1] + f_rrf[1] - f_rlb[1] - f_rrb[1]) * pos_x / sum_of_forces
            cop_r.point.x = max(min(cop_r.point.x, pos_x), -pos_x)
            cop_r.point.y = (f_rlf[1] + f_rlb[1] - f_rrf[1] - f_rrb[1]) * pos_y / sum_of_forces
            cop_r.point.y = max(min(cop_r.point.y, pos_y), -pos_y)
        else:
            cop_r.point.x = 0.0
            cop_r.point.y = 0.0
        self.cop_r_pub_.publish(cop_r)

    def publish_true_odom(self):
        position, orientation = self.simulation.get_robot_pose()
        self.odom_msg.pose.pose.position.x = position[0]
        self.odom_msg.pose.pose.position.y = position[1]
        self.odom_msg.pose.pose.position.z = position[2]
        self.odom_msg.pose.pose.orientation.x = orientation[0]
        self.odom_msg.pose.pose.orientation.y = orientation[1]
        self.odom_msg.pose.pose.orientation.z = orientation[2]
        self.odom_msg.pose.pose.orientation.w = orientation[3]
        self.true_odom_publisher.publish(self.odom_msg)

    def joint_goal_cb(self, msg: JointCommand):
        # only put new goals into the goal vector
        i = 0
        for name in msg.joint_names:
            self.simulation.joints[self.robot_index][name].set_position(msg.positions[i])
            i += 1

    def reset_cb(self, msg):
        self.simulation.reset()

    def parameters_callback(self, params):
        # we just get all parameters again, since it is easier
        #todo I think this does not really work like this?? needs testing
        self.node.get_logger().warn("Not sure if this parameter callback works.")
        self.simulation.set_dynamics(
            contact_damping=self.node.get_parameter("contact_damping").get_parameter_value().double_value,
            contact_stiffness=self.node.get_parameter("contact_stiffness").get_parameter_value().double_value,
            lateral_friction=self.node.get_parameter("lateral_friction").get_parameter_value().double_value,
            spinning_friction=self.node.get_parameter("spinning_friction").get_parameter_value().double_value,
            rolling_friction=self.node.get_parameter("rolling_friction").get_parameter_value().double_value,
            restitution=self.node.get_parameter("restitution").get_parameter_value().double_value)
        self.simulation.set_filter_params(self.node.get_parameter("cutoff").get_parameter_value().integer_value,
                                          self.node.get_parameter("order").get_parameter_value().integer_value)
