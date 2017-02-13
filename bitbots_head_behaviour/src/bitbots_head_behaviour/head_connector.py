import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from humanoid_league_msgs.msg import Role


class HeadConnector:
    def __init__(self):
        # params loading only once, for more performance later
        config = rospy.get_param("/Behaviour/Head/")
        self.delta = config["Search"]["headTurnPrecision"]
        self.wait_time = config["Search"]["headTurnTime"]
        self.pan_speed_max = config["Search"]["maxPanSpeedSearch"]
        self.tilt_speed_max = config["Search"]["maxTiltSpeedSearch"]

        # class variables
        self.current_pan_pos = 0
        self.current_tilt_pos = 0

        self.role = 0

        # preparing message for more performance
        self.pos_msg = JointTrajectory()
        self.pos_msg.joint_names = ["HeadPan", "HeadTilt"]
        self.point_msg = JointTrajectoryPoint()
        self.pos_msg.points = [self.point_msg]

        self.position_publisher = rospy.Publisher("/head_motor_goals", JointTrajectory, queue_size=10)
        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)
        rospy.Subscriber("/role", Role, self.role_cb)

    def send_motor_goals(self, pan_position: float, pan_speed: float, tilt_position: float, tilt_speed: float):
        self.point_msg.positions = [pan_position, tilt_position]
        self.point_msg.velocities = [pan_speed, tilt_speed]
        self.position_publisher.publish(self.pos_msg)

    def joint_state_cb(self, msg: JointState):
        i = 0
        for joint in msg.name:
            if joint == "HeadPan":
                self.current_pan_pos = msg.position[i]
            elif joint == "HeadTilt":
                self.current_tilt_pos = msg.position[i]
            i += 1

    def role_cb(self, msg):
        self.role = msg.role

    def get_current_head_pos(self):
        return self.current_pan_pos, self.current_tilt_pos

    def get_duty(self):
        return self.role