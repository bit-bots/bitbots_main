import time
import rospy
from bitbots_msgs.msg import FootPressure, JointCommand
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32, Bool


class ROSInterface:
    def __init__(self, simulation):
        rospy.init_node("pybullet_sim")

        self.simulation = simulation
        self.last_time = time.time()
        self.last_linear_vel = (0, 0, 0)

        # messages
        self.real_time_msg = Float32()
        self.joint_state_msg = JointState()
        self.joint_state_msg.header.frame_id = "base_link"
        self.joint_state_msg.name = self.simulation.initial_joints_positions.keys()
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

        # publisher
        self.left_foot_pressure_publisher = rospy.Publisher("foot_pressure_raw/left", FootPressure, queue_size=1)
        self.right_foot_pressure_publisher = rospy.Publisher("foot_pressure_raw/right", FootPressure, queue_size=1)
        self.joint_publisher = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.imu_publisher = rospy.Publisher("imu/data", Imu, queue_size=1)
        self.clock_publisher = rospy.Publisher("clock", Clock, queue_size=1)
        self.real_time_factor_publisher = rospy.Publisher("real_time_factor", Float32, queue_size=1)
        self.true_odom_publisher = rospy.Publisher("true_odom", Odometry, queue_size=1)

        # subscriber
        self.joint_goal_subscriber = rospy.Subscriber("DynamixelController/command", JointCommand, self.joint_goal_cb,
                                                      queue_size=1, tcp_nodelay=True)

        self.reset_subscriber = rospy.Subscriber("reset", Bool, self.reset_cb, queue_size=1, tcp_nodelay=True)

    def step(self):
        self.simulation.step()
        self.publish_joints()
        self.publish_imu()
        self.publish_foot_pressure()
        self.clock_msg.clock = rospy.Time.from_seconds(self.simulation.time)
        self.clock_publisher.publish(self.clock_msg)
        self.compute_real_time_factor()

    def run_simulation(self):
        while not rospy.is_shutdown():
            self.step()

    def compute_real_time_factor(self):
        time_now = time.time()
        self.real_time_msg.data = self.simulation.timestep / (time_now - self.last_time)
        self.last_time = time_now
        self.real_time_factor_publisher.publish(self.real_time_msg)

    def publish_joints(self):
        positions = []
        velocities = []
        efforts = []
        for name in self.joint_state_msg.name:
            joint = self.simulation.joints[name]
            positions.append(joint.get_position())
            velocities.append(joint.get_velocity())
            efforts.append(joint.get_torque())
        self.joint_state_msg.position = positions
        self.joint_state_msg.velocity = velocities
        self.joint_state_msg.effort = efforts
        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_publisher.publish(self.joint_state_msg)

    def publish_imu(self):
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
        self.imu_msg.linear_acceleration.x = linear_acc[0]
        self.imu_msg.linear_acceleration.y = linear_acc[0]
        self.imu_msg.linear_acceleration.z = linear_acc[0]
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_publisher.publish(self.imu_msg)

    def publish_foot_pressure(self):
        self.foot_msg_left.left_back = self.simulation.pressure_sensors["LLB"].get_force()
        self.foot_msg_left.left_front = self.simulation.pressure_sensors["LLF"].get_force()
        self.foot_msg_left.right_front = self.simulation.pressure_sensors["LRF"].get_force()
        self.foot_msg_left.right_back = self.simulation.pressure_sensors["LRB"].get_force()
        self.left_foot_pressure_publisher.publish(self.foot_msg_left)

        self.foot_msg_right.left_back = self.simulation.pressure_sensors["RLB"].get_force()
        self.foot_msg_right.left_front = self.simulation.pressure_sensors["RLF"].get_force()
        self.foot_msg_right.right_front = self.simulation.pressure_sensors["RRF"].get_force()
        self.foot_msg_right.right_back = self.simulation.pressure_sensors["RRB"].get_force()
        self.right_foot_pressure_publisher.publish(self.foot_msg_right)

    def publish_true_odom(self):
        position, orientation = self.simulation.get_robot_pose()
        self.odom_msg.pose.pose.position = position
        self.odom_msg.pose.pose.orientation = orientation
        self.true_odom_publisher.publish(self.odom_msg)

    def joint_goal_cb(self, msg: JointCommand):
        # only put new goals into the goal vector
        i = 0
        for name in msg.joint_names:
            self.simulation.joints[name].set_position(msg.positions[i])
            i += 1

    def reset_cb(self, msg):
        self.simulation.reset()
