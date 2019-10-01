import rospy
from bitbots_msgs.msg import JointCommand

# List of all joint names. Do not change the order as it is important for Gazebo
from humanoid_league_msgs.msg import Animation

JOINT_NAMES = ['HeadPan', 'HeadTilt', 'LShoulderPitch', 'LShoulderRoll', 'LElbow', 'RShoulderPitch',
               'RShoulderRoll', 'RElbow', 'LHipYaw', 'LHipRoll', 'LHipPitch', 'LKnee', 'LAnklePitch',
               'LAnkleRoll', 'RHipYaw', 'RHipRoll', 'RHipPitch', 'RKnee', 'RAnklePitch', 'RAnkleRoll']


class AnimationHcmBridge:
    def __init__(self):
        rospy.init_node("animation_hcm_bridge", anonymous=False)
        self.joint_publisher = rospy.Publisher("/DynamixelController/command", JointCommand, queue_size=10,
                                               tcp_nodelay=True)
        self.joint_command_msg = JointCommand()
        self.joint_command_msg.joint_names = JOINT_NAMES
        self.joint_command_msg.positions = [0] * 20
        self.joint_command_msg.velocities = [-1] * 20

        rospy.Subscriber("animation", Animation, self.animation_cb, queue_size=10, tcp_nodelay=True)

    def animation_cb(self, msg: Animation):
        self.joint_command_msg.header.stamp = rospy.Time.now()
        for i in range(len(msg.position.joint_names)):
            name = msg.position.joint_names[i]
            self.joint_command_msg.positions[JOINT_NAMES.index(name)] = msg.position.points[0].positions[i]
            self.joint_command_msg.velocities[JOINT_NAMES.index(name)] = -1

        self.joint_publisher.publish(self.joint_command_msg)
