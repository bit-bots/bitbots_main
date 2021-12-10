import time
import rospy
from bitbots_msgs.msg import FootPressure, JointCommand
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32, Bool
from tf.transformations import euler_from_quaternion

from wolfgang_pybullet_sim.cfg import simConfig
from dynamic_reconfigure.server import Server


class ROSInterface:
    def __init__(self, simulation, namespace='', node=True):
        self.namespace = namespace
        # give possibility to use the interface directly as class with setting node=False
        if node:
            if namespace == '':
                rospy.init_node("pybullet_sim")
            else:
                rospy.init_node('pybullet_sim', anonymous=True, argv=['clock:=/' + self.namespace + '/clock'])

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

        srv = Server(simConfig, self._dynamic_reconfigure_callback, namespace=namespace)

        # publisher
        self.left_foot_pressure_publisher = rospy.Publisher(self.namespace + "foot_pressure_left/raw", FootPressure,
                                                            queue_size=1)
        self.right_foot_pressure_publisher = rospy.Publisher(self.namespace + "foot_pressure_right/raw", FootPressure,
                                                             queue_size=1)
        self.left_foot_pressure_publisher_filtered = rospy.Publisher(self.namespace + "foot_pressure_left/filtered",
                                                                     FootPressure, queue_size=1)
        self.right_foot_pressure_publisher_filtered = rospy.Publisher(self.namespace + "foot_pressure_right/filtered",
                                                                      FootPressure, queue_size=1)
        self.joint_publisher = rospy.Publisher(self.namespace + "joint_states", JointState, queue_size=1)
        self.imu_publisher = rospy.Publisher(self.namespace + "imu/data", Imu, queue_size=1)
        self.clock_publisher = rospy.Publisher(self.namespace + "clock", Clock, queue_size=1)
        self.real_time_factor_publisher = rospy.Publisher(self.namespace + "real_time_factor", Float32, queue_size=1)
        self.true_odom_publisher = rospy.Publisher(self.namespace + "true_odom", Odometry, queue_size=1)
        self.cop_l_pub_ = rospy.Publisher(self.namespace + "cop_l", PointStamped, queue_size=1)
        self.cop_r_pub_ = rospy.Publisher(self.namespace + "cop_r", PointStamped, queue_size=1)

        # subscriber
        self.joint_goal_subscriber = rospy.Subscriber(self.namespace + "DynamixelController/command", JointCommand,
                                                      self.joint_goal_cb, queue_size=1, tcp_nodelay=True)

        self.reset_subscriber = rospy.Subscriber(self.namespace + "reset", Bool, self.reset_cb, queue_size=1,
                                                 tcp_nodelay=True)

    def step(self):
        self.simulation.step()
        self.publish_joints()
        self.publish_imu()
        self.publish_foot_pressure()
        self.publish_true_odom()
        self.clock_msg.clock = rospy.Time.from_seconds(self.simulation.time)
        self.clock_publisher.publish(self.clock_msg)
        self.compute_real_time_factor()

    def run_simulation(self, duration=None, sleep=0):
        start_time = rospy.get_time()
        while not rospy.is_shutdown() and (duration is None or rospy.get_time() - start_time < duration):
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
        self.joint_state_msg.header.stamp = rospy.Time.from_seconds(self.simulation.time)
        return self.joint_state_msg

    def publish_joints(self):
        self.joint_publisher.publish(self.get_joint_state_msg())

    def get_imu_msg(self):
        orientation = self.simulation.get_imu_quaternion()
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
        #adding gravity to the acceleration
        r,p,y = euler_from_quaternion(orientation)
        gravity = [r*9.81,p*9.81,y*9.81]
        linear_acc = tuple([linear_acc[0]+gravity[0], linear_acc[1]+gravity[1], linear_acc[2]+gravity[2]])
        self.imu_msg.linear_acceleration.x = linear_acc[0]
        self.imu_msg.linear_acceleration.y = linear_acc[1]
        self.imu_msg.linear_acceleration.z = linear_acc[2]
        self.imu_msg.header.stamp = rospy.Time.from_seconds(self.simulation.time)
        return self.imu_msg

    def publish_imu(self):
        self.imu_publisher.publish(self.get_imu_msg())

    def get_pressure_filtered_left(self):
        if len(self.simulation.pressure_sensors) == 0:
            rospy.logwarn_once("No pressure sensors found in simulation model")
            return self.foot_msg_left
        f_llb = self.simulation.pressure_sensors[self.robot_index]["LLB"].get_force()
        f_llf = self.simulation.pressure_sensors[self.robot_index]["LLF"].get_force()
        f_lrf = self.simulation.pressure_sensors[self.robot_index]["LRF"].get_force()
        f_lrb = self.simulation.pressure_sensors[self.robot_index]["LRB"].get_force()
        self.foot_msg_left.left_back = f_llb[1]
        self.foot_msg_left.left_front = f_llf[1]
        self.foot_msg_left.right_front = f_lrf[1]
        self.foot_msg_left.right_back = f_lrb[1]
        return self.foot_msg_left

    def get_pressure_filtered_right(self):
        if len(self.simulation.pressure_sensors) == 0:
            rospy.logwarn_once("No pressure sensors found in simulation model")
            return self.foot_msg_right
        f_rlb = self.simulation.pressure_sensors[self.robot_index]["RLB"].get_force()
        f_rlf = self.simulation.pressure_sensors[self.robot_index]["RLF"].get_force()
        f_rrf = self.simulation.pressure_sensors[self.robot_index]["RRF"].get_force()
        f_rrb = self.simulation.pressure_sensors[self.robot_index]["RRB"].get_force()
        self.foot_msg_right.left_back = f_rlb[1]
        self.foot_msg_right.left_front = f_rlf[1]
        self.foot_msg_right.right_front = f_rrf[1]
        self.foot_msg_right.right_back = f_rrb[1]
        return self.foot_msg_right

    def publish_foot_pressure(self):
        # some models dont have sensors
        if len(self.simulation.pressure_sensors) == 0:
            rospy.logwarn_once("No pressure sensors found in simulation model")
            return

        f_llb = self.simulation.pressure_sensors[self.robot_index]["LLB"].get_force()
        f_llf = self.simulation.pressure_sensors[self.robot_index]["LLF"].get_force()
        f_lrf = self.simulation.pressure_sensors[self.robot_index]["LRF"].get_force()
        f_lrb = self.simulation.pressure_sensors[self.robot_index]["LRB"].get_force()

        f_rlb = self.simulation.pressure_sensors[self.robot_index]["RLB"].get_force()
        f_rlf = self.simulation.pressure_sensors[self.robot_index]["RLF"].get_force()
        f_rrf = self.simulation.pressure_sensors[self.robot_index]["RRF"].get_force()
        f_rrb = self.simulation.pressure_sensors[self.robot_index]["RRB"].get_force()

        self.foot_msg_left.left_back = f_llb[0]
        self.foot_msg_left.left_front = f_llf[0]
        self.foot_msg_left.right_front = f_lrf[0]
        self.foot_msg_left.right_back = f_lrb[0]
        self.left_foot_pressure_publisher.publish(self.foot_msg_left)

        self.foot_msg_right.left_back = f_rlb[0]
        self.foot_msg_right.left_front = f_rlf[0]
        self.foot_msg_right.right_front = f_rrf[0]
        self.foot_msg_right.right_back = f_rrb[0]
        self.right_foot_pressure_publisher.publish(self.foot_msg_right)

        self.foot_msg_left.left_back = f_llb[1]
        self.foot_msg_left.left_front = f_llf[1]
        self.foot_msg_left.right_front = f_lrf[1]
        self.foot_msg_left.right_back = f_lrb[1]
        self.left_foot_pressure_publisher_filtered.publish(self.foot_msg_left)

        self.foot_msg_right.left_back = f_rlb[1]
        self.foot_msg_right.left_front = f_rlf[1]
        self.foot_msg_right.right_front = f_rrf[1]
        self.foot_msg_right.right_back = f_rrb[1]
        self.right_foot_pressure_publisher_filtered.publish(self.foot_msg_right)

        # center position on foot
        pos_x = 0.085
        pos_y = 0.045
        threshold = 0.0

        cop_l = PointStamped()
        cop_l.header.frame_id = "l_sole"
        cop_l.header.stamp = rospy.Time.from_seconds(self.simulation.time)
        sum_of_forces = f_llb[1] + f_llf[1] + f_lrf[1] + f_lrb[1]
        if sum_of_forces > threshold:
            cop_l.point.x = (f_llf[1] + f_lrf[1] - f_llb[1] - f_lrb[1]) * pos_x / sum_of_forces
            cop_l.point.x = max(min(cop_l.point.x, pos_x), -pos_x)
            cop_l.point.y = (f_llf[1] + f_llb[1] - f_lrf[1] - f_lrb[1]) * pos_y / sum_of_forces
            cop_l.point.y = max(min(cop_l.point.y, pos_y), -pos_y)
        else:
            cop_l.point.x = 0
            cop_l.point.y = 0
        self.cop_l_pub_.publish(cop_l)

        cop_r = PointStamped()
        cop_r.header.frame_id = "r_sole"
        cop_r.header.stamp = rospy.Time.from_seconds(self.simulation.time)
        sum_of_forces = f_rlb[1] + f_rlf[1] + f_rrf[1] + f_rrb[1]
        if sum_of_forces > threshold:
            cop_r.point.x = (f_rlf[1] + f_rrf[1] - f_rlb[1] - f_rrb[1]) * pos_x / sum_of_forces
            cop_r.point.x = max(min(cop_r.point.x, pos_x), -pos_x)
            cop_r.point.y = (f_rlf[1] + f_rlb[1] - f_rrf[1] - f_rrb[1]) * pos_y / sum_of_forces
            cop_r.point.y = max(min(cop_r.point.y, pos_y), -pos_y)
        else:
            cop_r.point.x = 0
            cop_r.point.y = 0
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

    def _dynamic_reconfigure_callback(self, config, level):
        self.simulation.set_foot_dynamics(config["contact_damping"], config["contact_stiffness"],
                                          config["joint_damping"], config["lateral_friction"],
                                          config["spinning_friction"], config["rolling_friction"])
        self.simulation.set_filter_params(config["cutoff"], config["order"])
        return config
