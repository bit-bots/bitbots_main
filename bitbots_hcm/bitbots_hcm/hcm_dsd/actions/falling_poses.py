from turtle import pos
from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from bitbots_hcm.hcm_dsd.hcm_blackboard import HcmBlackboard
from rclpy.duration import Duration
from bitbots_msgs.msg import JointCommand
from math import radians


def parse_dict_to_msg(joint_dict: dict, time_stamp):
    names = []
    positions = []
    for name, value in joint_dict.items():
        names.append(name)
        positions.append(radians(value))
    msg = JointCommand()
    msg.joint_names = names
    msg.positions = positions
    msg.velocities = len(names) * [-1]
    msg.accelerations = len(names) * [-1]
    msg.max_currents = len(names) * [-1]
    msg.header.stamp = time_stamp
    return msg


class AbstractFallingPose(AbstractActionElement):

    def __init__(self, blackboard: HcmBlackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.first_perform = True
        self.start_time = blackboard.node.get_clock().now()
        self.duration = parameters.get('duration', 1.0)
        self.falling_pose = None

    def perform(self, reevaluate=False):
        self.do_not_reevaluate()
        if self.first_perform:
            self.first_perform = False
            self.blackboard.joint_pub.publish(self.falling_pose)
        # wait some time
        if self.blackboard.node.get_clock().now() - self.start_time > Duration(seconds=self.duration):
            self.pop()


class FallingPoseFront(AbstractFallingPose):

    def __init__(self, blackboard: HcmBlackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        joint_dict = {
            "HeadPan": 0,
            "HeadTilt": 45,
            "LAnklePitch": -36,
            "LAnkleRoll": 4,
            "LElbow": 45,
            "LHipPitch": -11,
            "LHipRoll": 4,
            "LHipYaw": 6,
            "LKnee": 13,
            "LShoulderPitch": 90,
            "LShoulderRoll": 0,
            "RAnklePitch": 36,
            "RAnkleRoll": -4,
            "RElbow": -45,
            "RHipPitch": 11,
            "RHipRoll": -4,
            "RHipYaw": 6,
            "RKnee": -13,
            "RShoulderPitch": -90,
            "RShoulderRoll": 0
        }
        self.falling_pose = parse_dict_to_msg(joint_dict)


class FallingPoseBack(AbstractFallingPose):

    def __init__(self, blackboard: HcmBlackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        joint_dict = {
            "HeadPan": 0,
            "HeadTilt": 0,
            "LAnklePitch": -29,
            "LAnkleRoll": 0,
            "LElbow": 47,
            "LHipPitch": 51,
            "LHipRoll": 0,
            "LHipYaw": 0,
            "LKnee": 64,
            "LShoulderPitch": 1,
            "LShoulderRoll": 0,
            "RAnklePitch": 28,
            "RAnkleRoll": -4,
            "RElbow": -45,
            "RHipPitch": -50,
            "RHipRoll": -1,
            "RHipYaw": 1,
            "RKnee": -61,
            "RShoulderPitch": 0,
            "RShoulderRoll": 0
        }
        self.falling_pose = parse_dict_to_msg(joint_dict)


class FallingPoseLeft(AbstractFallingPose):

    def __init__(self, blackboard: HcmBlackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        joint_dict = {
            "HeadPan": 0,
            "HeadTilt": 0,
            "LAnklePitch": -26,
            "LAnkleRoll": 4,
            "LElbow": 44,
            "LHipPitch": 27,
            "LHipRoll": 4,
            "LHipYaw": -1,
            "LKnee": 58,
            "LShoulderPitch": -2,
            "LShoulderRoll": 0,
            "RAnklePitch": 26,
            "RAnkleRoll": -4,
            "RElbow": -42,
            "RHipPitch": -27,
            "RHipRoll": -4,
            "RHipYaw": 1,
            "RKnee": -58,
            "RShoulderPitch": 6,
            "RShoulderRoll": 0
        }
        self.falling_pose = parse_dict_to_msg(joint_dict)


class FallingPoseRight(AbstractFallingPose):

    def __init__(self, blackboard: HcmBlackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        joint_dict = {
            "HeadPan": 0,
            "HeadTilt": 0,
            "LAnklePitch": -26,
            "LAnkleRoll": 4,
            "LElbow": 44,
            "LHipPitch": 27,
            "LHipRoll": 4,
            "LHipYaw": -1,
            "LKnee": 58,
            "LShoulderPitch": -2,
            "LShoulderRoll": 0,
            "RAnklePitch": 26,
            "RAnkleRoll": -4,
            "RElbow": -42,
            "RHipPitch": -27,
            "RHipRoll": -4,
            "RHipYaw": 1,
            "RKnee": -58,
            "RShoulderPitch": 6,
            "RShoulderRoll": 0
        }
        self.falling_pose = parse_dict_to_msg(joint_dict)
